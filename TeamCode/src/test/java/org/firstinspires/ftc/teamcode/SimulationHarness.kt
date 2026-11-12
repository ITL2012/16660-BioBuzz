package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Pose2d
import java.io.File
import java.sql.DriverManager
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.max

/** How far a planner's expected value may drift before the golden check fails. */
private const val GOLDEN_VALUE_TOLERANCE = 0.75

/** Fixed seed for the planner's random playouts, so replays are repeatable. */
private const val HARNESS_SEED = 20260901

/** One planner decision, as the harness remembers it. */
data class PlannerDecisionRecord(
    val simTimeSec: Double,
    val action: String,
    val targetId: Int,
    val targetLabel: String,
    val expectedValue: Double,
    val estCostSec: Double,
    val rationale: String,
    val ballsHeld: Int,
    val timeRemainingSec: Double,
    val robotXcm: Double,
    val robotYcm: Double,
) {
    fun toSessionDecision(): SessionDecision = SessionDecision(
        tNanos = (simTimeSec * 1e9).toLong(),
        tMs = 0L,
        simTimeSec = simTimeSec,
        action = action,
        targetId = targetId,
        targetLabel = targetLabel,
        expectedValue = expectedValue,
        estCostSec = estCostSec,
        ballsHeld = ballsHeld,
        timeRemainingSec = timeRemainingSec,
        robotXcm = robotXcm,
        robotYcm = robotYcm,
        rationale = rationale,
    )
}

/**
 * Replays a recorded Black Box session and lets the planner react to it.
 *
 * The session is a SQLite file — the same file the Robot Controller writes.
 * Everything comes out of typed columns, so there is no JSON to parse and a
 * single odd row can never take the rest of the replay down: bad values are
 * skipped and reported instead.
 *
 * The golden master lives in a database too (the `golden` table), so a
 * regression check is a query, not a file parse.
 */
class SimulationHarness {
    var clockRateHz = 20.0
    var plannerRateHz = 2.0
    var visionConfidenceNoise = 0.0

    /**
     * When true the replay sleeps to match wall-clock time. Tests leave it
     * off: a 30 s session replays in milliseconds instead of half a minute.
     */
    var realtime = false

    private val world = WorldModel()
    /**
     * The planner runs on a FIXED SEED. A golden master is only worth
     * anything if replaying the same session reproduces the same decisions
     * — and MCTS playouts are random by nature.
     */
    private val planner = MCTSPlanner(
        playouts = 32,
        maxDepth = 3,
        random = kotlin.random.Random(HARNESS_SEED),
    )
    private val clock = PhaseClock(periodSec = 30.0, nowNanos = { simTimeNanos })
    private var simTimeNanos = 0L
    private var plannerAccumulatorSec = 0.0
    private var simPose = Pose2d(0.0, 0.0, 0.0)
    private var ballsHeld = 0

    private val decisions = mutableListOf<PlannerDecisionRecord>()
    private val events = mutableListOf<String>()

    /**
     * Loads a session database and replays it.
     *
     * [sessionName] picks the session out of the file; when it is absent,
     * the most recent session in the file is used.
     */
    fun loadSession(file: File, sessionName: String? = null): SimulationHarness {
        require(file.exists()) { "session database not found: $file" }
        val conn = DriverManager.getConnection("jdbc:sqlite:${file.absolutePath}")
        try {
            val store = SessionStore(JdbcSql(conn)).createSchema()
            val sessions = store.sessions()
            if (sessions.isEmpty()) {
                events.add("empty_session ${file.name}")
                return this
            }
            val header = sessionName?.let { name ->
                sessions.firstOrNull { (it["session_id"] as? String) == name }
            } ?: sessions.first()
            val sessionRef = (header["id"] as Number).toLong()
            replaySession(store, sessionRef)
        } finally {
            conn.close()
        }
        return this
    }

    /** One step of the replay timeline. */
    private sealed class Step {
        abstract val tNanos: Long
        class Pose(override val tNanos: Long, val row: Map<String, Any?>) : Step()
        class Tracks(override val tNanos: Long, val rows: List<Map<String, Any?>>) : Step()
        class Event(override val tNanos: Long, val type: String, val fields: Map<String, Any?>) : Step()
    }

    private fun replaySession(store: SessionStore, sessionRef: Long) {
        // --- build the timeline -------------------------------------------
        val steps = mutableListOf<Step>()
        for (row in store.states(sessionRef)) {
            val t = num(row, "t_nanos")?.toLong() ?: continue
            steps.add(Step.Pose(t, row))
        }
        val tracksByTime = store.tracks(sessionRef).groupBy { num(it, "t_nanos")?.toLong() ?: 0L }
        for ((t, rows) in tracksByTime) steps.add(Step.Tracks(t, rows))
        for ((row, fields) in store.events(sessionRef)) {
            val t = num(row, "t_nanos")?.toLong() ?: continue
            val type = text(row, "type") ?: continue
            steps.add(Step.Event(t, type, fields))
        }
        // Pose first, then the tracks it belongs to, then the events: the
        // sightings are expressed relative to the pose of that instant.
        steps.sortWith(
            compareBy<Step>(
                { it.tNanos },
                {
                    when (it) {
                        is Step.Pose -> 0
                        is Step.Tracks -> 1
                        is Step.Event -> 2
                    }
                },
            ),
        )

        // --- replay -------------------------------------------------------
        val firstNanos = steps.firstOrNull()?.tNanos ?: return
        for (step in steps) {
            val elapsedSec = (step.tNanos - firstNanos) / 1e9
            advanceTo(elapsedSec)
            when (step) {
                is Step.Pose -> applyPose(step.row, step.tNanos)
                is Step.Tracks -> applyTracks(step.rows, step.tNanos)
                is Step.Event -> applyEvent(step, elapsedSec)
            }
        }
        // Replay to the END of the recording. Using the wall clock here
        // would jump the simulation by however long ago the session was
        // logged — seconds, or years.
        val lastNanos = steps.lastOrNull()?.tNanos ?: return
        advanceTo((lastNanos - firstNanos) / 1e9)
    }

    private fun applyPose(row: Map<String, Any?>, tNanos: Long) {
        val px = num(row, "pose_x_cm")
        val py = num(row, "pose_y_cm")
        val hdg = num(row, "heading_rad")
        if (px == null || py == null || hdg == null) {
            events.add("bad_row state id=${row["id"]}: incomplete pose")
            return
        }
        simPose = Pose2d(px / 2.54, py / 2.54, hdg)

        // The hive is stored in field cm; the WorldModel wants a
        // robot-relative range/bearing, like the real vision pipeline.
        val hx = num(row, "hive_x_cm")
        val hy = num(row, "hive_y_cm")
        if (hx != null && hy != null) {
            val dx = hx - px
            val dy = hy - py
            val target = HiveTarget(
                tagId = -1,
                label = "RED SCORING",
                distanceCm = hypot(dx, dy),
                bearingDeg = Math.toDegrees(Math.atan2(dy, dx)) - Math.toDegrees(hdg),
                screenX = 0.0,
                screenY = 0.0,
                pixelDiagonal = 0.0,
                percentClusterFound = 100,
                fromFtcPose = false,
                timestampNanos = tNanos,
            )
            world.updateHive(target, px, py, hdg, tNanos)
        }
    }

    private fun applyTracks(rows: List<Map<String, Any?>>, tNanos: Long) {
        val px = num(rows.firstOrNull() ?: return, "pose_x_cm")
        // The tracks are in FIELD coordinates; convert them into the
        // robot-relative sightings the WorldModel expects.
        val robotX = simPose.position.x * 2.54
        val robotY = simPose.position.y * 2.54
        val hdg = simPose.heading.toDouble()

        val sightings = mutableListOf<Sighting>()
        for (row in rows) {
            val kind = text(row, "kind")
            val x = num(row, "x_cm")
            val y = num(row, "y_cm")
            if (kind == null || x == null || y == null) {
                // A row that does not add up is skipped and reported — it
                // must never take the rest of the session down with it.
                events.add("bad_row track id=${row["id"]}: unusable position")
                continue
            }
            val label = when (kind) {
                "opponent" -> "Robot"
                "flower" -> "Flower"
                else -> text(row, "label") ?: "Ball"
            }
            val bearing = bearingTo(x, y, robotX, robotY, hdg)
            val distance = hypot(x - robotX, y - robotY)
            sightings.add(
                Sighting(
                    label = label,
                    distanceCm = distance,
                    bearingDeg = bearing,
                    contourArea = if (kind == "opponent") 5000.0 else 1000.0,
                    isBall = kind == "ball",
                ),
            )
        }
        if (sightings.isNotEmpty()) {
            // ALL the sightings of this frame go in ONE update. Feeding them
            // one at a time would make the tracker see a single target per
            // frame, which is exactly the case the tracker should resolve.
            world.update(sightings, robotX, robotY, hdg, tNanos)
        }
    }

    private fun applyEvent(step: Step.Event, elapsedSec: Double) {
        val type = step.type
        when (type) {
            "cycle_start" -> events.add("cycle_start at ${fmt(elapsedSec)}s")
            "phase" -> events.add("phase ${text(step.fields, "name") ?: "?"} at ${fmt(elapsedSec)}s")
            "pickup" -> {
                ballsHeld++
                events.add(
                    "pickup ${text(step.fields, "label") ?: "?"} " +
                        "via ${text(step.fields, "how") ?: "?"} at ${fmt(elapsedSec)}s",
                )
            }
            "fire_done" -> {
                val fired = int(step.fields, "fired") ?: 0
                ballsHeld = max(0, ballsHeld - fired)
                events.add("fire $fired at ${fmt(elapsedSec)}s")
            }
            "session_start" -> events.add("session_start ${text(step.fields, "label") ?: "?"} at ${fmt(elapsedSec)}s")
        }
    }

    private fun bearingTo(x: Double, y: Double, px: Double, py: Double, hdg: Double): Double {
        return Math.toDegrees(Math.atan2(y - py, x - px)) - Math.toDegrees(hdg)
    }

    private fun advanceTo(targetSec: Double) {
        val targetNanos = (targetSec * 1e9).toLong()
        val dtSec = 1.0 / clockRateHz
        val plannerPeriodSec = 1.0 / plannerRateHz
        while (simTimeNanos < targetNanos) {
            simTimeNanos += (dtSec * 1e9).toLong()
            plannerAccumulatorSec += dtSec

            if (plannerAccumulatorSec >= plannerPeriodSec) {
                plannerAccumulatorSec -= plannerPeriodSec
                val remaining = clock.remainingSec()
                if (remaining > 0) {
                    val px = simPose.position.x * 2.54
                    val py = simPose.position.y * 2.54
                    val d = planner.decide(world, ballsHeld, clock.remainingSec(), px, py)
                    decisions.add(
                        PlannerDecisionRecord(
                            simTimeSec = elapsedSec(),
                            action = d.action.name,
                            targetId = d.targetId,
                            targetLabel = d.targetLabel,
                            expectedValue = d.expectedValue,
                            estCostSec = d.estCostSec,
                            rationale = d.rationale,
                            ballsHeld = ballsHeld,
                            timeRemainingSec = remaining,
                            robotXcm = px,
                            robotYcm = py,
                        ),
                    )
                }
            }
            if (realtime) Thread.sleep((dtSec * 1000).toLong())
        }
    }

    fun elapsedSec(): Double {
        return simTimeNanos / 1e9
    }

    fun run(): SimulationHarness {
        return this
    }

    fun printSummary() {
        println("Decisions: ${decisions.size}")
    }

    /** The recorded session events (phase changes, pickups, bad rows). */
    fun events(): List<String> = events.toList()

    /** How many planner decisions the replay produced. */
    fun decisionCount(): Int = decisions.size

    /** The decisions the replay produced. */
    fun decisions(): List<PlannerDecisionRecord> = decisions.toList()

    /**
     * Stores the replay's decisions as a golden master inside a database.
     * The same file can hold several goldens; [name] tells them apart.
     */
    fun exportGoldenMaster(
        dbFile: File,
        name: String = "default",
        sessionId: String = "auto",
        toleranceSec: Double = 0.5,
    ): SimulationHarness {
        dbFile.parentFile?.mkdirs()
        val conn = DriverManager.getConnection("jdbc:sqlite:${dbFile.absolutePath}")
        try {
            val store = SessionStore(JdbcSql(conn)).createSchema()
            store.putGolden(
                name = name,
                sessionId = sessionId,
                decisions = decisions.map { it.toSessionDecision() },
                toleranceSec = toleranceSec,
            )
        } finally {
            conn.close()
        }
        return this
    }

    /**
     * Compares the replay against a golden master stored in a database.
     * Throws an [AssertionError] listing what drifted.
     */
    fun assertPlannerMatchesGolden(
        dbFile: File,
        name: String = "default",
        toleranceSec: Double = 0.5,
    ): SimulationHarness {
        require(dbFile.exists()) { "golden master not found: $dbFile" }
        val conn = DriverManager.getConnection("jdbc:sqlite:${dbFile.absolutePath}")
        val golden: List<PlannerDecisionRecord>
        try {
            val store = SessionStore(JdbcSql(conn)).createSchema()
            val header = store.goldens().firstOrNull { (it["name"] as? String) == name }
                ?: throw IllegalArgumentException("golden master not found: $name in ${dbFile.name}")
            val goldenRef = (header["id"] as Number).toLong()
            golden = store.goldenDecisions(goldenRef).map { row ->
                PlannerDecisionRecord(
                    simTimeSec = num(row, "sim_time_sec") ?: 0.0,
                    action = text(row, "action") ?: "",
                    targetId = int(row, "target_id") ?: -1,
                    targetLabel = text(row, "target_label") ?: "",
                    expectedValue = num(row, "expected_value") ?: 0.0,
                    estCostSec = num(row, "est_cost_sec") ?: 0.0,
                    rationale = text(row, "rationale") ?: "",
                    ballsHeld = int(row, "balls_held") ?: 0,
                    timeRemainingSec = num(row, "time_remaining_sec") ?: 0.0,
                    robotXcm = num(row, "robot_x_cm") ?: 0.0,
                    robotYcm = num(row, "robot_y_cm") ?: 0.0,
                )
            }
        } finally {
            conn.close()
        }

        val problems = mutableListOf<String>()
        if (golden.size != decisions.size) {
            problems.add(
                "decision count differs: golden=${golden.size}, replay=${decisions.size}",
            )
        }
        val n = minOf(golden.size, decisions.size)
        for (i in 0 until n) {
            val g = golden[i]
            val r = decisions[i]
            if (g.action != r.action) {
                problems.add("#$i action differs: golden=${g.action}, replay=${r.action}")
            }
            if (g.targetId != r.targetId) {
                problems.add(
                    "#$i target differs: golden=${g.targetId}(${g.targetLabel}), " +
                        "replay=${r.targetId}(${r.targetLabel})",
                )
            }
            if (abs(g.simTimeSec - r.simTimeSec) > toleranceSec) {
                problems.add(
                    "#$i timing differs by ${abs(g.simTimeSec - r.simTimeSec)} s " +
                        "(> $toleranceSec s): golden=${g.simTimeSec}, replay=${r.simTimeSec}",
                )
            }
            if (abs(g.expectedValue - r.expectedValue) > GOLDEN_VALUE_TOLERANCE) {
                problems.add(
                    "#$i value differs: golden=${g.expectedValue}, replay=${r.expectedValue}",
                )
            }
            if (problems.size >= 12) break
        }
        if (problems.isNotEmpty()) {
            throw AssertionError(
                "planner drifted from the golden master ($name):\n" +
                    problems.joinToString("\n") { "  $it" },
            )
        }
        return this
    }

    private fun fmt(v: Double): String = String.format("%.2f", v)

    companion object {
        /**
         * Reads a number out of a row. A column that holds something that
         * is not a number reads as null instead of throwing — one odd row
         * must not stop the replay.
         */
        fun num(row: Map<String, Any?>, key: String): Double? = when (val v = row[key]) {
            null -> null
            is Number -> v.toDouble()
            is Boolean -> if (v) 1.0 else 0.0
            is String -> v.trim().toDoubleOrNull()
            else -> null
        }

        /** Reads an integer out of a row, tolerating a numeric string. */
        fun int(row: Map<String, Any?>, key: String): Int? = num(row, key)?.toInt()

        /** Reads text out of a row, tolerating a number. */
        fun text(row: Map<String, Any?>, key: String): String? = when (val v = row[key]) {
            null -> null
            is String -> v
            else -> v.toString()
        }
    }
}
