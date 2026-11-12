package org.firstinspires.ftc.teamcode

/**
 * ============ SESSION DB — one SQLite file per Black Box session ============
 *
 * The Black Box used to write a JSON-lines state log plus a folder of JPEG
 * frames. Both were brittle: a single truncated line made the whole replay
 * unreadable, and the frames were thousands of loose files that had to be
 * copied over USB as a directory tree.
 *
 * Now a session is ONE SQLite file. Everything about it — the robot state,
 * the world-model tracks, the planner decisions, the events, AND the camera
 * frames (as BLOBs) — lives in typed tables inside that one file.
 *
 * WHY SQLITE:
 *  * The Robot Controller writes it with Android's own SQLite (no extra
 *    dependency on the robot).
 *  * A computer reads the very same file with SQLite JDBC — or with Python's
 *    stdlib sqlite3, which needs no pip install at all.
 *  * No JSON parsing anywhere: every column has a type, so a malformed
 *    record can never take the rest of the session down with it.
 *
 * The schema lives here so both sides agree on it byte for byte. See
 * [SessionSchema] for the DDL, [SessionStore] for the shared read/write
 * logic, and [Sql] for the tiny abstraction that lets the same code run
 * against Android's SQLite and against JDBC.
 */

/** The BLOB of a camera frame: its JPEG bytes plus what we know about it. */
data class SessionFrame(
    val camera: String,
    val captureNanos: Long,
    val jpeg: ByteArray,
    val widthPx: Int = 0,
    val heightPx: Int = 0,
    /** Wall-clock capture time (ms) — lets a reader line a frame up with the
     *  events that happened around it. */
    val tMs: Long = 0L,
) {
    override fun equals(other: Any?): Boolean =
        other is SessionFrame && camera == other.camera && captureNanos == other.captureNanos &&
            jpeg.contentEquals(other.jpeg)

    override fun hashCode(): Int =
        (camera.hashCode() * 31 + captureNanos.hashCode()) * 31 + jpeg.contentHashCode()
}

/** One world-model track as it was seen at one instant. */
data class SessionTrack(
    val kind: String, // "ball" | "opponent" | "flower"
    val trackId: Int,
    val xCm: Double,
    val yCm: Double,
    val label: String = "",
    val confidence: Double = 0.0,
    val velXcmPerSec: Double = 0.0,
    val velYcmPerSec: Double = 0.0,
    val intent: String = "",
    val intentConfidence: Double = 0.0,
)

/** The robot's own state at one instant. */
data class SessionState(
    val tNanos: Long,
    val tMs: Long,
    val poseXcm: Double = 0.0,
    val poseYcm: Double = 0.0,
    val headingRad: Double = 0.0,
    val hiveXcm: Double? = null,
    val hiveYcm: Double? = null,
    val hiveConfidence: Double = 0.0,
    val ballsHeld: Int = -1,
    val flywheelLeftRpm: Double = -1.0,
    val flywheelRightRpm: Double = -1.0,
    val batteryVolts: Double = -1.0,
)

/** One planner decision, fully typed. */
data class SessionDecision(
    val tNanos: Long,
    val tMs: Long,
    val simTimeSec: Double = 0.0,
    val action: String = "",
    val targetId: Int = -1,
    val targetLabel: String = "",
    val expectedValue: Double = 0.0,
    val estCostSec: Double = 0.0,
    val ballsHeld: Int = 0,
    val timeRemainingSec: Double = 0.0,
    val robotXcm: Double = 0.0,
    val robotYcm: Double = 0.0,
    val rationale: String = "",
    val source: String = "",
)

/**
 * A discrete event. [fields] holds the event's payload as name/value pairs
 * instead of a JSON blob — the values are stored in typed columns, so a
 * reader can filter on them with plain SQL.
 */
data class SessionEvent(
    val tNanos: Long,
    val tMs: Long,
    val type: String,
    val fields: List<Pair<String, Any?>> = emptyList(),
)

/**
 * A tiny SQL gateway. Android's SQLite and JDBC disagree about almost
 * everything except the SQL itself, so the shared code talks to this
 * interface instead of to a driver.
 */
interface Sql {
    /** Runs a statement that returns no rows. */
    fun update(sql: String, args: List<Any?> = emptyList())

    /** Runs a query and returns the rows as name -> value maps. */
    fun query(sql: String, args: List<Any?> = emptyList()): List<Map<String, Any?>>

    /** Runs a statement that must produce exactly one integer (e.g. an id). */
    fun insertReturningId(sql: String, args: List<Any?> = emptyList()): Long
}

/**
 * The schema. Both sides run [DDL] on open, so a file written by the robot
 * and a file written by the desktop tools have exactly the same shape.
 *
 * Every table is keyed by [SESSION_REF] so several sessions can live in one
 * file (the desktop side keeps a library of sessions and goldens in a
 * single .db).
 */
object SessionSchema {

    /** Bumped whenever the tables change shape. */
    const val VERSION = 1

    const val SESSION_REF = "session_ref"

    val DDL: List<String> = listOf(
        // --- the session header -------------------------------------------
        """
        CREATE TABLE IF NOT EXISTS session (
            id            INTEGER PRIMARY KEY AUTOINCREMENT,
            session_id    TEXT    NOT NULL UNIQUE,
            label         TEXT    NOT NULL DEFAULT '',
            schema_version INTEGER NOT NULL DEFAULT 1,
            created_at_ms INTEGER NOT NULL DEFAULT 0,
            started_nanos INTEGER NOT NULL DEFAULT 0,
            ended_nanos   INTEGER NOT NULL DEFAULT 0,
            notes         TEXT    NOT NULL DEFAULT ''
        )
        """,
        // --- the robot's own state, one row per snapshot --------------------
        """
        CREATE TABLE IF NOT EXISTS state (
            id            INTEGER PRIMARY KEY AUTOINCREMENT,
            ${SessionSchema.SESSION_REF}  INTEGER NOT NULL,
            t_nanos       INTEGER NOT NULL,
            t_ms          INTEGER NOT NULL,
            pose_x_cm     REAL    NOT NULL DEFAULT 0,
            pose_y_cm     REAL    NOT NULL DEFAULT 0,
            heading_rad   REAL    NOT NULL DEFAULT 0,
            hive_x_cm     REAL,
            hive_y_cm     REAL,
            hive_conf     REAL    NOT NULL DEFAULT 0,
            balls_held    INTEGER NOT NULL DEFAULT -1,
            fly_l_rpm     REAL    NOT NULL DEFAULT -1,
            fly_r_rpm     REAL    NOT NULL DEFAULT -1,
            battery_v     REAL    NOT NULL DEFAULT -1
        )
        """,
        // --- the world-model tracks, one row per track per snapshot ---------
        """
        CREATE TABLE IF NOT EXISTS track (
            id            INTEGER PRIMARY KEY AUTOINCREMENT,
            ${SessionSchema.SESSION_REF}  INTEGER NOT NULL,
            t_nanos       INTEGER NOT NULL,
            kind          TEXT    NOT NULL,
            track_id      INTEGER NOT NULL,
            x_cm          REAL    NOT NULL DEFAULT 0,
            y_cm          REAL    NOT NULL DEFAULT 0,
            label         TEXT    NOT NULL DEFAULT '',
            confidence    REAL    NOT NULL DEFAULT 0,
            vel_x         REAL    NOT NULL DEFAULT 0,
            vel_y         REAL    NOT NULL DEFAULT 0,
            intent        TEXT    NOT NULL DEFAULT '',
            intent_conf   REAL    NOT NULL DEFAULT 0
        )
        """,
        // --- the planner decisions -----------------------------------------
        """
        CREATE TABLE IF NOT EXISTS decision (
            id            INTEGER PRIMARY KEY AUTOINCREMENT,
            ${SessionSchema.SESSION_REF}  INTEGER NOT NULL,
            t_nanos       INTEGER NOT NULL,
            t_ms          INTEGER NOT NULL,
            sim_time_sec  REAL    NOT NULL DEFAULT 0,
            action        TEXT    NOT NULL DEFAULT '',
            target_id     INTEGER NOT NULL DEFAULT -1,
            target_label  TEXT    NOT NULL DEFAULT '',
            expected_value REAL   NOT NULL DEFAULT 0,
            est_cost_sec  REAL    NOT NULL DEFAULT 0,
            balls_held    INTEGER NOT NULL DEFAULT 0,
            time_remaining_sec REAL NOT NULL DEFAULT 0,
            robot_x_cm    REAL    NOT NULL DEFAULT 0,
            robot_y_cm    REAL    NOT NULL DEFAULT 0,
            rationale     TEXT    NOT NULL DEFAULT '',
            source        TEXT    NOT NULL DEFAULT ''
        )
        """,
        // --- the discrete events (phase changes, pickups, jams...) -----------
        """
        CREATE TABLE IF NOT EXISTS event (
            id            INTEGER PRIMARY KEY AUTOINCREMENT,
            ${SessionSchema.SESSION_REF}  INTEGER NOT NULL,
            seq           INTEGER NOT NULL DEFAULT 0,
            t_nanos       INTEGER NOT NULL,
            t_ms          INTEGER NOT NULL,
            type          TEXT    NOT NULL
        )
        """,
        // --- the event payload: name/value, typed, no JSON ------------------
        """
        CREATE TABLE IF NOT EXISTS event_field (
            event_ref     INTEGER NOT NULL,
            key           TEXT    NOT NULL,
            value_text    TEXT,
            value_num     REAL,
            value_int     INTEGER,
            PRIMARY KEY (event_ref, key)
        )
        """,
        // --- the camera frames, as BLOBS ------------------------------------
        """
        CREATE TABLE IF NOT EXISTS frame (
            id            INTEGER PRIMARY KEY AUTOINCREMENT,
            ${SessionSchema.SESSION_REF}  INTEGER NOT NULL,
            capture_nanos INTEGER NOT NULL,
            t_ms          INTEGER NOT NULL DEFAULT 0,
            camera        TEXT    NOT NULL,
            bytes         INTEGER NOT NULL DEFAULT 0,
            width_px      INTEGER NOT NULL DEFAULT 0,
            height_px     INTEGER NOT NULL DEFAULT 0,
            data          BLOB
        )
        """,
        // --- the golden masters ----------------------------------------------
        """
        CREATE TABLE IF NOT EXISTS golden (
            id            INTEGER PRIMARY KEY AUTOINCREMENT,
            name          TEXT    NOT NULL UNIQUE,
            session_id    TEXT    NOT NULL DEFAULT '',
            created_at_ms INTEGER NOT NULL DEFAULT 0,
            decision_count INTEGER NOT NULL DEFAULT 0,
            tolerance_sec REAL    NOT NULL DEFAULT 0.5,
            notes         TEXT    NOT NULL DEFAULT ''
        )
        """,
        """
        CREATE TABLE IF NOT EXISTS golden_decision (
            id            INTEGER PRIMARY KEY AUTOINCREMENT,
            golden_ref    INTEGER NOT NULL,
            seq           INTEGER NOT NULL,
            sim_time_sec  REAL    NOT NULL DEFAULT 0,
            action        TEXT    NOT NULL DEFAULT '',
            target_id     INTEGER NOT NULL DEFAULT -1,
            target_label  TEXT    NOT NULL DEFAULT '',
            expected_value REAL   NOT NULL DEFAULT 0,
            est_cost_sec  REAL    NOT NULL DEFAULT 0,
            balls_held    INTEGER NOT NULL DEFAULT 0,
            time_remaining_sec REAL NOT NULL DEFAULT 0,
            robot_x_cm    REAL    NOT NULL DEFAULT 0,
            robot_y_cm    REAL    NOT NULL DEFAULT 0,
            rationale     TEXT    NOT NULL DEFAULT ''
        )
        """,
        // --- the indexes the readers actually use ----------------------------
        "CREATE INDEX IF NOT EXISTS idx_state_session ON state(" + SESSION_REF + ", t_nanos)",
        "CREATE INDEX IF NOT EXISTS idx_track_session ON track(" + SESSION_REF + ", t_nanos)",
        "CREATE INDEX IF NOT EXISTS idx_event_session ON event(" + SESSION_REF + ", seq)",
        "CREATE INDEX IF NOT EXISTS idx_decision_session ON decision(" + SESSION_REF + ", t_nanos)",
        "CREATE INDEX IF NOT EXISTS idx_frame_session ON frame(" + SESSION_REF + ", capture_nanos)",
        "CREATE INDEX IF NOT EXISTS idx_golden_decision ON golden_decision(golden_ref, seq)",
    )

    /** Creates every table. Safe to call on an existing file. */
    fun create(sql: Sql) {
        for (statement in DDL) sql.update(statement)
    }
}

/**
 * Reads and writes sessions. Shared by the robot (through Android's SQLite)
 * and by the desktop tools (through JDBC), so both sides agree on the file
 * format by construction.
 */
class SessionStore(private val sql: Sql) {

    /** Creates the schema if it is not there yet. */
    fun createSchema(): SessionStore {
        SessionSchema.create(sql)
        return this
    }

    /**
     * Starts a session. Returns its row id. Re-using an existing
     * [sessionId] replaces that session's rows, so a re-recorded session
     * does not pile up beside the old one.
     */
    fun beginSession(
        sessionId: String,
        label: String = "",
        createdAtMs: Long = System.currentTimeMillis(),
        startedNanos: Long = 0L,
    ): Long {
        val existing = sql.query(
            "SELECT id FROM session WHERE session_id = ?",
            listOf(sessionId),
        ).firstOrNull()
        if (existing != null) {
            val id = (existing["id"] as Number).toLong()
            clearSession(id)
            sql.update(
                "UPDATE session SET label = ?, schema_version = ?, created_at_ms = ?, " +
                    "started_nanos = ?, ended_nanos = 0, notes = '' WHERE id = ?",
                listOf(label, SessionSchema.VERSION, createdAtMs, startedNanos, id),
            )
            return id
        }
        return sql.insertReturningId(
            "INSERT INTO session (session_id, label, schema_version, created_at_ms, started_nanos) " +
                "VALUES (?, ?, ?, ?, ?)",
            listOf(sessionId, label, SessionSchema.VERSION, createdAtMs, startedNanos),
        )
    }

    /** Wipes a session's rows (except the header) so it can be re-recorded. */
    fun clearSession(sessionRef: Long) {
        for (table in listOf("state", "track", "decision", "event", "frame")) {
            sql.update("DELETE FROM $table WHERE ${SessionSchema.SESSION_REF} = ?", listOf(sessionRef))
        }
        sql.update("DELETE FROM event_field WHERE event_ref NOT IN (SELECT id FROM event)")
    }

    /** Stamps the end of a session. */
    fun endSession(sessionRef: Long, endedNanos: Long) {
        sql.update("UPDATE session SET ended_nanos = ? WHERE id = ?", listOf(endedNanos, sessionRef))
    }

    /** Appends one robot-state snapshot. */
    fun addState(sessionRef: Long, s: SessionState) {
        sql.update(
            "INSERT INTO state (${SessionSchema.SESSION_REF}, t_nanos, t_ms, pose_x_cm, pose_y_cm, heading_rad, " +
                "hive_x_cm, hive_y_cm, hive_conf, balls_held, fly_l_rpm, fly_r_rpm, battery_v) " +
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
            listOf(
                sessionRef, s.tNanos, s.tMs, s.poseXcm, s.poseYcm, s.headingRad,
                s.hiveXcm, s.hiveYcm, s.hiveConfidence, s.ballsHeld,
                s.flywheelLeftRpm, s.flywheelRightRpm, s.batteryVolts,
            ),
        )
    }

    /** Appends the world-model tracks of one snapshot. */
    fun addTracks(sessionRef: Long, tNanos: Long, tracks: List<SessionTrack>) {
        for (t in tracks) {
            sql.update(
                "INSERT INTO track (${SessionSchema.SESSION_REF}, t_nanos, kind, track_id, x_cm, y_cm, label, " +
                    "confidence, vel_x, vel_y, intent, intent_conf) " +
                    "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                listOf(
                    sessionRef, tNanos, t.kind, t.trackId, t.xCm, t.yCm, t.label,
                    t.confidence, t.velXcmPerSec, t.velYcmPerSec, t.intent, t.intentConfidence,
                ),
            )
        }
    }

    /** Appends one planner decision. */
    fun addDecision(sessionRef: Long, d: SessionDecision) {
        sql.update(
            "INSERT INTO decision (${SessionSchema.SESSION_REF}, t_nanos, t_ms, sim_time_sec, action, target_id, " +
                "target_label, expected_value, est_cost_sec, balls_held, time_remaining_sec, " +
                "robot_x_cm, robot_y_cm, rationale, source) " +
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
            listOf(
                sessionRef, d.tNanos, d.tMs, d.simTimeSec, d.action, d.targetId,
                d.targetLabel, d.expectedValue, d.estCostSec, d.ballsHeld,
                d.timeRemainingSec, d.robotXcm, d.robotYcm, d.rationale, d.source,
            ),
        )
    }

    /** Appends one event and its payload. */
    fun addEvent(sessionRef: Long, seq: Int, e: SessionEvent) {
        val eventId = sql.insertReturningId(
            "INSERT INTO event (${SessionSchema.SESSION_REF}, seq, t_nanos, t_ms, type) VALUES (?, ?, ?, ?, ?)",
            listOf(sessionRef, seq, e.tNanos, e.tMs, e.type),
        )
        for ((key, value) in e.fields) {
            sql.update(
                "INSERT OR REPLACE INTO event_field (event_ref, key, value_text, value_num, value_int) " +
                    "VALUES (?, ?, ?, ?, ?)",
                listOf(eventId, key, asText(value), asNum(value), asInt(value)),
            )
        }
    }

    /** Appends one camera frame, JPEG bytes and all. */
    fun addFrame(sessionRef: Long, f: SessionFrame) {
        sql.update(
            "INSERT INTO frame (${SessionSchema.SESSION_REF}, capture_nanos, t_ms, camera, bytes, width_px, height_px, data) " +
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
            listOf(
                sessionRef, f.captureNanos, f.tMs, f.camera, f.jpeg.size, f.widthPx, f.heightPx, f.jpeg,
            ),
        )
    }

    // --- reading ---------------------------------------------------------

    /** The session headers, newest first. */
    fun sessions(): List<Map<String, Any?>> =
        sql.query("SELECT * FROM session ORDER BY created_at_ms DESC, id DESC")

    /** The state snapshots of a session, in time order. */
    fun states(sessionRef: Long): List<Map<String, Any?>> =
        sql.query(
            "SELECT * FROM state WHERE ${SessionSchema.SESSION_REF} = ? ORDER BY t_nanos, id",
            listOf(sessionRef),
        )

    /** The tracks of a session, in time order. */
    fun tracks(sessionRef: Long): List<Map<String, Any?>> =
        sql.query(
            "SELECT * FROM track WHERE ${SessionSchema.SESSION_REF} = ? ORDER BY t_nanos, id",
            listOf(sessionRef),
        )

    /** The decisions of a session, in time order. */
    fun decisions(sessionRef: Long): List<Map<String, Any?>> =
        sql.query(
            "SELECT * FROM decision WHERE ${SessionSchema.SESSION_REF} = ? ORDER BY t_nanos, id",
            listOf(sessionRef),
        )

    /** The events of a session, in the order they were logged. */
    fun events(sessionRef: Long): List<Pair<Map<String, Any?>, Map<String, Any?>>> {
        val events = sql.query(
            "SELECT * FROM event WHERE $SESSION_REF = ? ORDER BY seq, id",
            listOf(sessionRef),
        )
        if (events.isEmpty()) return emptyList()
        val fields = sql.query("SELECT * FROM event_field")
        val byEvent = fields.groupBy { (it["event_ref"] as Number).toLong() }
        return events.map { ev ->
            val id = (ev["id"] as Number).toLong()
            // One map per event, holding every field it carried.
            val fieldMap = byEvent[id].orEmpty().associate { row ->
                val key = row["key"] as? String ?: ""
                val value = row["value_int"] ?: row["value_num"] ?: row["value_text"]
                key to value
            }
            ev to fieldMap
        }
    }

    /** The frames of a session, in capture order. */
    fun frames(sessionRef: Long): List<Map<String, Any?>> =
        sql.query(
            "SELECT * FROM frame WHERE ${SessionSchema.SESSION_REF} = ? ORDER BY capture_nanos, id",
            listOf(sessionRef),
        )

    // --- golden masters ---------------------------------------------------

    /**
     * Stores a golden master: the planner decisions a replay must reproduce.
     * Re-storing the same name replaces the old one.
     */
    fun putGolden(
        name: String,
        sessionId: String,
        decisions: List<SessionDecision>,
        toleranceSec: Double = 0.5,
        createdAtMs: Long = System.currentTimeMillis(),
        notes: String = "",
    ): Long {
        sql.update("DELETE FROM golden_decision WHERE golden_ref IN (SELECT id FROM golden WHERE name = ?)", listOf(name))
        sql.update("DELETE FROM golden WHERE name = ?", listOf(name))
        val goldenRef = sql.insertReturningId(
            "INSERT INTO golden (name, session_id, created_at_ms, decision_count, tolerance_sec, notes) " +
                "VALUES (?, ?, ?, ?, ?, ?)",
            listOf(name, sessionId, createdAtMs, decisions.size, toleranceSec, notes),
        )
        decisions.forEachIndexed { seq, d ->
            sql.update(
                "INSERT INTO golden_decision (golden_ref, seq, sim_time_sec, action, target_id, " +
                    "target_label, expected_value, est_cost_sec, balls_held, time_remaining_sec, " +
                    "robot_x_cm, robot_y_cm, rationale) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                listOf(
                    goldenRef, seq, d.simTimeSec, d.action, d.targetId, d.targetLabel,
                    d.expectedValue, d.estCostSec, d.ballsHeld, d.timeRemainingSec,
                    d.robotXcm, d.robotYcm, d.rationale,
                ),
            )
        }
        return goldenRef
    }

    /** The golden masters, newest first. */
    fun goldens(): List<Map<String, Any?>> =
        sql.query("SELECT * FROM golden ORDER BY created_at_ms DESC, id DESC")

    /** The decisions of a golden master, in order. */
    fun goldenDecisions(goldenRef: Long): List<Map<String, Any?>> =
        sql.query(
            "SELECT * FROM golden_decision WHERE golden_ref = ? ORDER BY seq, id",
            listOf(goldenRef),
        )

    companion object {
        /** The column every per-session table is keyed by. */
        const val SESSION_REF = SessionSchema.SESSION_REF

        /** How a value is stored in the text column. */
        fun asText(value: Any?): String? = when (value) {
            null -> null
            is String -> value
            is Boolean -> value.toString()
            else -> value.toString()
        }

        /** How a value is stored in the numeric column. */
        fun asNum(value: Any?): Double? = when (value) {
            null -> null
            is Number -> value.toDouble()
            is Boolean -> if (value) 1.0 else 0.0
            else -> value.toString().toDoubleOrNull()
        }

        /** How a value is stored in the integer column. */
        fun asInt(value: Any?): Long? = when (value) {
            null -> null
            is Number -> value.toLong()
            is Boolean -> if (value) 1L else 0L
            else -> value.toString().toDoubleOrNull()?.toLong()
        }
    }
}
