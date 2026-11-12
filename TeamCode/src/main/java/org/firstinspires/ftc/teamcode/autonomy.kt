package org.firstinspires.ftc.teamcode

/**
 * ============ AUTONOMY CORE — shared by master and the superhuman branch ============
 *
 * The shared decision substrate — IDENTICAL to master's autonomy.kt, plus
 * the superhuman-only additions:
 *  * [PhaseClock]        — the 30s autonomous time budget          (core)
 *  * [ShooterModel]      — the calibrated RPM-vs-distance curve   (core)
 *  * [WorldModel]        — persistent field memory (tracks + hive) (core)
 *  * [StrategicPlanner]  — lookahead + endgame value/cost brain   (superhuman)
 *
 * Pure Kotlin — no Android imports — so the JVM unit tests
 * (src/test/java) can exercise all of it directly.
 */

import java.util.Locale
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin

// ============================================================================
// SECTION 1: PhaseClock — the 30s autonomous budget manager
// ============================================================================


/**
 * The phase machine's own timeouts can sum to more than the autonomous
 * period (9s collect + 6s flower + 4s polish + 8s align + fire > 30s), which
 * historically meant the FMS could kill the OpMode mid-fire and un-parked.
 * Every phase now asks the clock for a deadline instead of trusting its own
 * timeout: budgets are clamped to what remains after the reserved work
 * (park, fire), and the score loop asks [fits] before starting a cycle that
 * cannot finish.
 *
 * Reserves can be static (pass reserveSec) or dynamic (provide a
 * [reserveCalculator] lambda that computes reserves from live state).
 */
class PhaseClock(
    private val periodSec: Double = AUTONOMY_PERIOD_SEC,
    private val nowNanos: () -> Long = System::nanoTime,
    /** Optional dynamic reserve calculator. If provided, [deadlineNanos] with
     * reserveSec=0 will use this instead of the static reserve. */
    private val reserveCalculator: (() -> Double)? = null,
) {
    /**
     * The time source, kept under a different name than the [nowNanos]
     * accessor below: a member function and a constructor property cannot
     * share a name without one of them calling itself forever.
     */
    private val timeSource = nowNanos

    private val startNanos = timeSource()

    /** Raw clock reading (nanos) on the same timebase as [nowNanos]. */
    fun nowNanos(): Long = timeSource()

    fun elapsedSec(): Double = (timeSource() - startNanos) / 1e9

    /** Seconds left in the autonomous period. */
    fun remainingSec(): Double = (periodSec - elapsedSec()).coerceAtLeast(0.0)

    /** True when at least [sec] seconds remain in the period. */
    fun fits(sec: Double): Boolean = remainingSec() >= sec

    /**
     * Deadline (nanos) for a phase with its own [phaseTimeoutMs] cap: never
     * more than the phase wants, never more than the period has left after
     * reserves are set aside for later work. Zero-length means "start
     * nothing" (the deadline is now).
     *
     * If [reserveCalculator] was provided in the constructor, pass 0 for
     * [reserveSec] to use dynamic reserves. Otherwise uses the static [reserveSec].
     */
    fun deadlineNanos(phaseTimeoutMs: Double, reserveSec: Double = 0.0): Long {
        val effectiveReserve = if (reserveSec == 0.0 && reserveCalculator != null) {
            reserveCalculator()
        } else {
            reserveSec
        }
        val availableMs = (remainingSec() - effectiveReserve) * 1e3
        if (availableMs <= 0.0) return timeSource()
        return timeSource() + (minOf(phaseTimeoutMs, availableMs) * 1e6).toLong()
    }

    /** Gets the current effective reserve (for telemetry/debugging). */
    fun currentReserveSec(): Double = reserveCalculator?.invoke() ?: 0.0
}

/**
 * ============ DYNAMIC RESERVES — what the endgame is going to need ============
 *
 * The PhaseClock only works if we honestly reserve the time the endgame
 * needs. A flat "reserve 7 seconds" is wrong in both directions: on the
 * field's far side from the hive the robot needs a 10+ second run home, and
 * while parked in a corner it needs almost nothing.
 *
 * This calculator derives the reserve from live state:
 *
 *  * FIRE reserve = travel time to the hive + flywheel spin-up + one feed
 *    stroke per ball we actually hold + an alignment buffer. Holding four
 *    balls costs twice as much feed time as holding one, and a long run home
 *    costs more travel time — both scale automatically.
 *  * PARK reserve = the back-off we still owe at the end. When the robot is
 *    deep in the field it needs a moment to drive out of the way of the
 *    scoring traffic; when it is already against a wall the back-off is
 *    nearly free.
 *
 * The two are summed and handed to [PhaseClock] as its reserve, so every
 * phase deadline is clamped by what the endgame will really cost.
 */
class DynamicReserveCalculator(
    /** Current robot position in field cm, or null when unknown. */
    private val robotPositionCm: () -> Pair<Double, Double>?,
    /** Best known hive position in field cm, or null when never seen. */
    private val hivePositionCm: () -> Pair<Double, Double>?,
    /** How many balls the robot currently holds. */
    private val ballsHeld: () -> Int,
    /** Distance to the nearest wall in cm (or null when unknown). */
    private val distanceToWallCm: () -> Double? = { null },
    /** True while the robot is lined up on the hive. */
    private val isAligned: () -> Boolean = { false },
) {

    /** True when a hive fix exists, so the fire reserve can be exact. */
    fun hasHiveFix(): Boolean = hivePositionCm() != null

    /**
     * Seconds reserved for the final scoring run: travel to the hive,
     * spin-up, feeding every held ball, and a small alignment buffer.
     */
    fun fireReserveSec(): Double {
        val robot = robotPositionCm()
        val hive = hivePositionCm()
        val travel = if (robot != null && hive != null) {
            hypot(hive.first - robot.first, hive.second - robot.second) * RESERVE_TRAVEL_SEC_PER_CM
        } else {
            // Unknown hive: assume a pessimistic cross-field run so we never
            // start a phase we cannot pay back.
            150.0 * RESERVE_TRAVEL_SEC_PER_CM
        }
        val balls = ballsHeld().coerceIn(0, MAX_BALLS_HELD)
        val feed = balls * RESERVE_FIRE_PER_BALL_SEC
        return (travel + RESERVE_FIRE_BASE_SEC + feed).coerceAtMost(RESERVE_FIRE_MAX_SEC)
    }

    /**
     * Seconds reserved for the final park. Small when the robot is already
     * near a wall (the back-off is quick), larger in the open field.
     */
    fun parkReserveSec(): Double {
        val wallDist = distanceToWallCm()
        val base = PLANNER_PARK_SEC
        val extra = if (wallDist != null && wallDist > RESERVE_PARK_DISTANT_CM) {
            RESERVE_PARK_DISTANT_SEC
        } else {
            0.0
        }
        return (base + extra).coerceAtMost(RESERVE_PARK_MAX_SEC)
    }

    /**
     * Seconds reserved for lining up on the hive.
     *
     * Once the robot IS lined up this drops to zero: the alignment has
     * happened, and reserving the time again would make the robot believe
     * it cannot afford to fire.
     */
    fun alignReserveSec(): Double = if (isAligned()) 0.0 else ALIGN_ESTIMATE_SEC

    /**
     * The total reserve the clock must hold back.
     *
     * This is the WHOLE endgame: line up, drive home, spin up, feed every
     * ball we hold, park. Every other phase is work done ON TOP of it, so
     * a phase deadline can never eat into the time the endgame needs.
     */
    fun totalReserveSec(): Double = alignReserveSec() + fireReserveSec() + parkReserveSec()

    /** Human-readable breakdown for telemetry. */
    fun describe(): String =
        "align %.1fs + fire %.1fs (balls=%d) + park %.1fs = %.1fs".format(
            alignReserveSec(),
            fireReserveSec(),
            ballsHeld(),
            parkReserveSec(),
            totalReserveSec(),
        )
}

// ============================================================================
// SECTION 2: ShooterModel (was shooter.kt)
// ============================================================================

/**
 * ============ SHOOTER MODEL — flywheel RPM vs. landing distance ============
 *
 * The calibration curve lives in globals.kt: [SHOOTER_CALIBRATION_RPM] paired
 * with [SHOOTER_CALIBRATION_DISTANCE_CM] (measured with the "Flywheel
 * Calibration" OpMode + tools/flywheel_analyzer.py). This object interpolates
 * that curve so the robot can ask "what flywheel RPM lands the ball at X cm?"
 * for ANY distance — the hive AprilTag estimator ([HiveScoring]) supplies X
 * live during matches.
 *
 * Until you fill in real measured distances, [isCalibrated] is false and every
 * query falls back to full speed (the old "spin at 100%" behavior), so the
 * robot is never left unable to shoot.
 */
object ShooterModel {

    /** Valid (rpm, distanceCm) samples, sorted by RPM ascending. */
    private fun samples(): List<Pair<Double, Double>> =
        SHOOTER_CALIBRATION_RPM.zip(SHOOTER_CALIBRATION_DISTANCE_CM)
            .filter { it.second > 0.0 }
            .sortedBy { it.first }

    /** True once at least two real calibration points exist. */
    fun isCalibrated(): Boolean = samples().size >= 2

    /** Shortest distance the curve covers (closest useful shooting distance). */
    fun minRangeCm(): Double = samples().minOfOrNull { it.second } ?: 0.0

    /** Longest distance the curve covers (the flywheel's physical reach). */
    fun maxRangeCm(): Double = samples().maxOfOrNull { it.second } ?: 0.0

    /**
     * Farthest distance we can shoot: the calibrated curve's reach, or the
     * HiveScoringConfig fallback window when no calibration data exists yet.
     */
    fun effectiveMaxRangeCm(): Double =
        if (isCalibrated()) maxRangeCm() else HiveScoringConfig.SHOOT_MAX_RANGE_CM

    /**
     * Flywheel RPM needed to land the ball [distanceCm] away. Linear
     * interpolation between the nearest calibrated samples; distances outside
     * the calibrated range clamp to the nearest end (too close still uses the
     * slowest calibrated RPM, too far uses the fastest). Uncalibrated -> 6000.
     */
    fun rpmForDistance(distanceCm: Double): Double {
        val s = samples()
        if (s.size < 2) return FLYWHEEL_MOTOR_MAX_RPM

        var lo = s.first()
        var hi = s.last()
        for (i in 0 until s.size - 1) {
            if (distanceCm in s[i].second..s[i + 1].second) {
                lo = s[i]
                hi = s[i + 1]
                break
            }
        }
        if (distanceCm <= lo.second) return lo.first
        if (distanceCm >= hi.second) return hi.first

        val t = (distanceCm - lo.second) / (hi.second - lo.second)
        return lo.first + t * (hi.first - lo.first)
    }

    /** Live summary for telemetry / the Driver Station. */
    fun summary(): String =
        if (isCalibrated()) {
            String.format(Locale.US, "calibrated %.0f-%.0fcm", minRangeCm(), maxRangeCm())
        } else {
            "UNCALIBRATED (fires full speed)"
        }
}

// ============================================================================
// SECTION 4: WorldModel (was WorldModel.kt)
// ============================================================================

data class Sighting(
    val label: String,
    /** Straight-line distance from the robot, cm. */
    val distanceCm: Double,
    /** + = target is to the RIGHT of the robot centerline, degrees. */
    val bearingDeg: Double,
    val contourArea: Double,
    val isBall: Boolean,
)

/** Opponent intent states for HMM. */
enum class OpponentIntent {
    SCORING,      // Moving toward hive, likely to score
    DEFENDING,    // Near our balls/hive, blocking
    COLLECTING,   // Moving toward balls/flowers
    TRANSIT,      // Moving between zones, no clear intent
    UNKNOWN       // Insufficient data
}

/** Flower state for depletion/cooldown tracking. */
enum class FlowerState {
    FULL,         // Never pulled, full nectar available
    DEPLETED,     // Recently pulled, in cooldown
    REGENERATING, // Cooldown ending, nectar returning
    UNKNOWN       // State unclear
}

/** Flower cluster for multi-flower route optimization. */
data class FlowerCluster(
    val id: Int,
    var flowers: MutableList<TrackedObject>,
    val centerXcm: Double,
    val centerYcm: Double,
    val totalNectarValue: Double,  // sum of NECTAR_VALUE per flower
    var state: FlowerState = FlowerState.FULL,
    var lastPulledNanos: Long = 0L,
    var pullCount: Int = 0,
) {
    fun distanceTo(xCm: Double, yCm: Double): Double = hypot(centerXcm - xCm, centerYcm - yCm)
    val flowerCount: Int get() = flowers.size
}

/**
 * Depletion record for a SPOT ON THE FIELD where a flower was emptied.
 *
 * The per-flower [FlowerDepletion] record is keyed by track id, but a track
 * id dies with its track: when the camera loses the flower and finds it
 * again, it is a NEW track with a NEW id, and the cooldown would be lost.
 * That is the same reason the cluster set cannot carry the state (it is
 * rebuilt every second).
 *
 * A patch is keyed by WHERE it is, not by what it is called, so it survives
 * both re-clustering and re-identification. The robot has to come back to
 * the same physical spot to get the nectar anyway.
 */
data class DepletedPatch(
    val xcm: Double,
    val ycm: Double,
    var pullCount: Int = 1,
    var lastPulledNanos: Long = 0L,
) {
    /** Cooldown still owed on this spot (0 when it is usable again). */
    fun cooldownRemainingSec(nowNanos: Long): Double {
        val since = (nowNanos - lastPulledNanos) / 1e9
        val required = FLOWER_COOLDOWN_BASE_SEC + pullCount * FLOWER_COOLDOWN_PER_PULL_SEC
        return (required - since).coerceAtLeast(0.0)
    }

    /** True when the spot is still worth avoiding. */
    fun onCooldown(nowNanos: Long): Boolean = cooldownRemainingSec(nowNanos) > 0.0
}

/**
 * Depletion record for a SINGLE flower.
 *
 * Cluster-level state is not enough: the cluster set is rebuilt every second
 * from whatever flowers are currently visible, so a cluster can lose its
 * "depleted" flag the moment a re-cluster happens. This record is keyed by
 * the flower's track id and therefore survives re-clustering, which is what
 * stops the robot from re-pulling the same flower five seconds after it
 * emptied it.
 */
data class FlowerDepletion(
    val flowerId: Int,
    var pullCount: Int = 0,
    var lastPulledNanos: Long = 0L,
    var state: FlowerState = FlowerState.FULL,
) {
    /** Seconds of cooldown still owed on this flower (0 when it is usable). */
    fun cooldownRemainingSec(nowNanos: Long): Double {
        if (state == FlowerState.FULL || state == FlowerState.UNKNOWN) return 0.0
        val since = (nowNanos - lastPulledNanos) / 1e9
        val required = FLOWER_COOLDOWN_BASE_SEC + pullCount * FLOWER_COOLDOWN_PER_PULL_SEC
        return when (state) {
            FlowerState.DEPLETED -> (required - since).coerceAtLeast(0.0)
            FlowerState.REGENERATING -> (required + FLOWER_REGEN_SEC - since).coerceAtLeast(0.0)
            else -> 0.0
        }
    }

    /** True when the flower may be pulled right now. */
    fun isAvailable(nowNanos: Long): Boolean = cooldownRemainingSec(nowNanos) <= 0.0
}

/** Static field obstacle (hive posts, walls, etc.) for path planning avoidance. */
data class FieldObstacle(
    val id: Int,
    val xCm: Double,
    val yCm: Double,
    val radiusCm: Double,
    val label: String,
    val isHivePost: Boolean = false,
) {
    fun distanceTo(xCm: Double, yCm: Double): Double = max(0.0, hypot(this.xCm - xCm, this.yCm - yCm) - radiusCm)
    fun blocksPath(fromX: Double, fromY: Double, toX: Double, toY: Double, robotRadiusCm: Double = 25.0): Boolean {
        // Line-circle intersection test
        val dx = toX - fromX
        val dy = toY - fromY
        val fx = fromX - xCm
        val fy = fromY - yCm
        val a = dx * dx + dy * dy
        val b = 2.0 * (fx * dx + fy * dy)
        val c = (fx * fx + fy * fy) - (radiusCm + robotRadiusCm) * (radiusCm + robotRadiusCm)
        val discriminant = b * b - 4.0 * a * c
        if (discriminant < 0.0) return false
        val t1 = (-b - Math.sqrt(discriminant)) / (2.0 * a)
        val t2 = (-b + Math.sqrt(discriminant)) / (2.0 * a)
        return (t1 >= 0.0 && t1 <= 1.0) || (t2 >= 0.0 && t2 <= 1.0)
    }
}

data class TrackedObject(
    val id: Int,
    val label: String,
    var fieldXcm: Double,
    var fieldYcm: Double,
    var confidence: Double,
    var lastSeenNanos: Long,
    var velXcmPerSec: Double,
    var velYcmPerSec: Double,
    val isBall: Boolean,
    /** Latest sighting's contour area (px^2) — the chase-quality gate. */
    var lastAreaPx: Double = 0.0,
    // Opponent intent HMM (only populated for opponent tracks)
    var intent: OpponentIntent = OpponentIntent.UNKNOWN,
    var intentConfidence: Double = 0.0,
    var intentSinceNanos: Long = 0L,
) {
    /** Game value of this object (POLLEN_VALUE / NECTAR_VALUE; 0 for others). */
    val value: Double
        get() = when {
            label.contains("Nectar") -> NECTAR_VALUE
            isBall -> POLLEN_VALUE
            else -> 0.0
        }

    fun distanceTo(xCm: Double, yCm: Double): Double = hypot(fieldXcm - xCm, fieldYcm - yCm)

    /** Seconds since the last visual sighting. */
    fun ageSec(nowNanos: Long): Double = (nowNanos - lastSeenNanos) / 1e9

    /**
     * Distance (cm) and bearing (deg, + = right of the robot centerline) of
     * this track from the robot — the same convention as live sightings, so
     * one set of pursuit gains serves visible and remembered targets alike.
     */
    fun relativeToRobot(robotXcm: Double, robotYcm: Double, headingRad: Double): Pair<Double, Double> {
        val dx = fieldXcm - robotXcm
        val dy = fieldYcm - robotYcm
        val bearing = wrapDeg(Math.toDegrees(headingRad) - Math.toDegrees(atan2(dy, dx)))
        return hypot(dx, dy) to bearing
    }
}

/** Wraps an angle in degrees into (-180, 180]. */
private fun wrapDeg(a: Double): Double = ((a + 180.0) % 360.0 + 360.0) % 360.0 - 180.0

class WorldModel {

    private val tracks = mutableListOf<TrackedObject>()
    private var nextId = 1

    // Flower cluster management (dynamic clustering + depletion tracking)
    private val flowerClusters = mutableListOf<FlowerCluster>()
    private var nextClusterId = 1
    private var lastClusterRebuildNanos = 0L

    /**
     * Per-flower depletion records, keyed by track id. These outlive the
     * cluster set (which is rebuilt every second), so a flower stays in
     * cooldown even after it is re-clustered.
     */
    private val flowerDepletion = mutableMapOf<Int, FlowerDepletion>()

    /**
     * Spots on the field where nectar was taken, keyed by position rather
     * than by track id. This is the record that actually keeps the robot
     * away from an emptied flower: track ids are re-issued every time the
     * camera re-identifies a flower, and clusters are rebuilt every second,
     * so neither can carry the cooldown across a re-identification.
     */
    private val depletedPatches = mutableListOf<DepletedPatch>()

    /** One particle filter per tracked opponent. */
    private val opponentParticles = OpponentParticleBank()

    /**
     * Incremented every time the flower-cluster set changes. The planner
     * (and the phase scheduler) watch this to know when a mid-match
     * re-evaluation produced a new best cluster.
     */
    var flowerClusterRevision: Int = 0
        private set

    /** Timestamp of the last cluster rebuild (nanos). */
    fun lastClusterRebuildNanos(): Long = lastClusterRebuildNanos

    // Static field obstacles (hive posts, perimeter walls, etc.)
    private val fieldObstacles = mutableListOf<FieldObstacle>()

    // Hive memory (from the AprilTag estimator — the most trusted source).
    private var hiveXcm = 0.0
    private var hiveYcm = 0.0
    private var hiveConfidence = 0.0
    private var hiveSeenNanos = 0L
    private var lastUpdateNanos = 0L

    init {
        // Initialize static field obstacles
        initFieldObstacles()
    }

    /**
     * Fuses the latest robot-relative [sightings] into the world model.
     * Pass the fused pose in cm ([poseXcm]/[poseYcm], [headingRad] CCW) when
     * available (autonomous / SelfPlay); pass nulls in pose-less teleop.
     */
    fun update(
        sightings: List<Sighting>,
        poseXcm: Double?,
        poseYcm: Double?,
        headingRad: Double,
        nowNanos: Long,
    ) {
        val dtSec = if (lastUpdateNanos == 0L) 0.0 else (nowNanos - lastUpdateNanos) / 1e9
        lastUpdateNanos = nowNanos

        if (dtSec > 0) {
            val decay = exp(-dtSec / WM_CONFIDENCE_TAU_SEC)
            for (t in tracks) t.confidence *= decay
        }

        val used = HashSet<Int>()
        for (s in sightings) {
            // robot-relative body frame: +forward, +left (bearing + = right)
            val bRad = Math.toRadians(s.bearingDeg)
            val fCm = s.distanceCm * cos(bRad)
            val lCm = -s.distanceCm * sin(bRad)

            // to field frame when we have a pose
            val xCm = if (poseXcm != null && poseYcm != null)
                poseXcm + fCm * cos(headingRad) - lCm * sin(headingRad) else fCm
            val yCm = if (poseXcm != null && poseYcm != null)
                poseYcm + fCm * sin(headingRad) + lCm * cos(headingRad) else lCm

            // nearest same-label track within the association radius
            var best: TrackedObject? = null
            var bestD = WM_TRACK_MATCH_CM
            for (t in tracks) {
                if (t.label != s.label || t.id in used) continue
                val d = hypot(t.fieldXcm - xCm, t.fieldYcm - yCm)
                if (d < bestD) {
                    best = t; bestD = d
                }
            }

            if (best != null) {
                val trackDt = (nowNanos - best.lastSeenNanos) / 1e9
                if (trackDt > 0.02 && poseXcm != null) {
                    best.velXcmPerSec = 0.7 * best.velXcmPerSec + 0.3 * (xCm - best.fieldXcm) / trackDt
                    best.velYcmPerSec = 0.7 * best.velYcmPerSec + 0.3 * (yCm - best.fieldYcm) / trackDt
                }
                // EMA-smooth the position so pinhole noise doesn't jerk the
                // track (the velocity estimate above still uses the raw delta).
                val a = WM_TRACK_EMA_ALPHA
                best.fieldXcm += a * (xCm - best.fieldXcm)
                best.fieldYcm += a * (yCm - best.fieldYcm)
                best.confidence = (best.confidence * 0.5 + 0.7).coerceAtMost(1.0)
                best.lastSeenNanos = nowNanos
                best.lastAreaPx = s.contourArea
                // Update opponent intent HMM for opponent tracks
                if (!best.isBall && best.label != "Flower") {
                    // Opponents get the particle-filter treatment: the cloud
                    // fuses the raw detection smoothly and survives the gaps
                    // where the camera loses them.
                    //
                    // The VELOCITY stays on the track's own EMA above. That
                    // estimate is deliberately responsive (it has to be: the
                    // dodge logic extrapolates only a fraction of a second
                    // ahead), while the particle cloud is deliberately
                    // smooth — blending the two would give a velocity that
                    // is both lagged AND half-strength.
                    val filter = opponentParticles.filterFor(best.id)
                    if (trackDt > 0.0) filter.predict(trackDt)
                    filter.setVelocityHint(best.velXcmPerSec, best.velYcmPerSec)
                    filter.update(xCm, yCm)
                    val (pxEst, pyEst) = filter.estimatePosition()
                    val fa = OPP_PARTICLE_FUSE_ALPHA
                    best.fieldXcm += fa * (pxEst - best.fieldXcm)
                    best.fieldYcm += fa * (pyEst - best.fieldYcm)
                    updateOpponentIntent(best, poseXcm ?: 0.0, poseYcm ?: 0.0, nowNanos)
                }
                used.add(best.id)
            } else {
                tracks.add(
                    TrackedObject(
                        id = nextId++, label = s.label,
                        fieldXcm = xCm, fieldYcm = yCm,
                        confidence = 0.6, lastSeenNanos = nowNanos,
                        velXcmPerSec = 0.0, velYcmPerSec = 0.0,
                        isBall = s.isBall,
                        lastAreaPx = s.contourArea,
                    )
)
        }
    }

        // forget stale / faded tracks
        tracks.removeAll { t ->
            val ageSec = (nowNanos - t.lastSeenNanos) / 1e9
            val maxAge = if (t.isBall) WM_BALL_MAX_AGE_SEC else WM_ROBOT_MAX_AGE_SEC
            ageSec > maxAge || t.confidence < 0.05
        }

        // Drop particle filters for tracks that no longer exist.
        opponentParticles.prune(tracks.map { it.id }.toSet())

        // Rebuild flower clusters on a short cadence so the planner always
        // sees the CURRENT best patch — a flower that gets pulled, or a new
        // patch that comes into view, changes the answer within a second.
        val rebuildIntervalNanos = (FLOWER_CLUSTER_REBUILD_SEC * 1e9).toLong()
        if (nowNanos - lastClusterRebuildNanos > rebuildIntervalNanos) {
            rebuildFlowerClusters(nowNanos)
            lastClusterRebuildNanos = nowNanos
        }

        // Update flower cluster states (depletion/cooldown/regeneration)
        updateFlowerClusterStates(nowNanos)

        // Update per-flower cooldown states (they outlive the cluster set).
        updateFlowerDepletionStates(nowNanos)

        if (hiveSeenNanos != 0L && dtSec > 0) hiveConfidence *= exp(-dtSec / WM_CONFIDENCE_TAU_SEC)
    }

    /** Fuses the AprilTag hive measurement (already the best-range estimate). */
    fun updateHive(hive: HiveTarget?, poseXcm: Double?, poseYcm: Double?, headingRad: Double, nowNanos: Long) {
        val t = hive ?: return
        val bRad = Math.toRadians(t.bearingDeg)
        val fCm = t.distanceCm * cos(bRad)
        val lCm = -t.distanceCm * sin(bRad)
        if (poseXcm != null && poseYcm != null) {
            hiveXcm = poseXcm + fCm * cos(headingRad) - lCm * sin(headingRad)
            hiveYcm = poseYcm + fCm * sin(headingRad) + lCm * cos(headingRad)
            // Update hive post positions when we have a good hive detection
            if (t.fromFtcPose && t.percentClusterFound > 50) {
                updateHivePosts(hiveXcm, hiveYcm, headingRad)
            }
        }
        hiveConfidence = if (hiveConfidence > 0) (hiveConfidence * 0.5 + 0.8).coerceAtMost(1.0) else 0.8
        hiveSeenNanos = nowNanos
    }

    /**
     * Updates opponent intent using a simple HMM based on position, velocity,
     * and proximity to key field elements (hive, balls, our robot).
     */
    private fun updateOpponentIntent(
        track: TrackedObject,
        robotXcm: Double,
        robotYcm: Double,
        nowNanos: Long,
    ) {
        val hive = hive()
        val dtSec = if (track.intentSinceNanos == 0L) 1.0 else (nowNanos - track.intentSinceNanos) / 1e9
        track.intentSinceNanos = nowNanos

        // Compute features for intent classification
        val speed = hypot(track.velXcmPerSec, track.velYcmPerSec)
        val distToHive = if (hive != null) hypot(hive.first - track.fieldXcm, hive.second - track.fieldYcm) else 200.0
        val distToRobot = hypot(track.fieldXcm - robotXcm, track.fieldYcm - robotYcm)
        val radialVelToHive = if (hive != null && distToHive > 1.0) {
            val dx = hive.first - track.fieldXcm
            val dy = hive.second - track.fieldYcm
            (dx * track.velXcmPerSec + dy * track.velYcmPerSec) / distToHive
        } else 0.0
        val radialVelToRobot = if (distToRobot > 1.0) {
            val dx = robotXcm - track.fieldXcm
            val dy = robotYcm - track.fieldYcm
            (dx * track.velXcmPerSec + dy * track.velYcmPerSec) / distToRobot
        } else 0.0

        // Find nearest ball
        var nearestBallDist = 200.0
        for (b in balls()) {
            nearestBallDist = min(nearestBallDist, b.distanceTo(track.fieldXcm, track.fieldYcm))
        }

        // HMM: compute emission probabilities for each intent
        val emissions = DoubleArray(OpponentIntent.values().size)
        
        // SCORING: near hive, moving toward hive
        emissions[OpponentIntent.SCORING.ordinal] = when {
            distToHive < 80.0 && radialVelToHive < -20.0 -> 0.9
            distToHive < 120.0 && radialVelToHive < -10.0 -> 0.7
            distToHive < 150.0 && radialVelToHive < 0.0 -> 0.4
            else -> 0.05
        }
        
        // DEFENDING: near our robot or our balls, low speed
        emissions[OpponentIntent.DEFENDING.ordinal] = when {
            distToRobot < 100.0 && speed < 30.0 -> 0.85
            distToRobot < 150.0 && nearestBallDist < 80.0 && speed < 50.0 -> 0.6
            else -> 0.05
        }
        
        // COLLECTING: near balls, moving toward them
        emissions[OpponentIntent.COLLECTING.ordinal] = when {
            nearestBallDist < 80.0 && speed > 20.0 -> 0.8
            nearestBallDist < 120.0 && speed > 10.0 -> 0.5
            else -> 0.05
        }
        
        // TRANSIT: moving fast, not near anything specific
        emissions[OpponentIntent.TRANSIT.ordinal] = when {
            speed > 60.0 && distToHive > 150.0 && nearestBallDist > 120.0 && distToRobot > 150.0 -> 0.7
            speed > 40.0 -> 0.4
            else -> 0.1
        }
        
        // UNKNOWN: fallback
        emissions[OpponentIntent.UNKNOWN.ordinal] = 0.1

        // Transition matrix (prior): tendency to stay in same state
        val transition = DoubleArray(OpponentIntent.values().size)
        for (i in OpponentIntent.values().indices) {
            transition[i] = if (i == track.intent.ordinal) 0.7 else 0.3 / (OpponentIntent.values().size - 1)
        }

        // Forward algorithm: belief = normalize(emission * transition * prev_belief)
        // Simplified: just use emission * persistence
        var bestIntent = track.intent
        var bestScore = -1.0
        for (i in OpponentIntent.values().indices) {
            val persistence = if (i == track.intent.ordinal) 1.2 else 1.0
            val score = emissions[i] * transition[i] * persistence
            if (score > bestScore) {
                bestScore = score
                bestIntent = OpponentIntent.values()[i]
            }
        }

        // Update with hysteresis: require confidence to switch
        if (bestIntent != track.intent) {
            if (bestScore > track.intentConfidence + 0.15) {
                track.intent = bestIntent
                track.intentConfidence = bestScore
            }
        } else {
            track.intentConfidence = track.intentConfidence * 0.9 + bestScore * 0.1
        }
    }

    /** Remembered balls, best-confidence first. */
    fun balls(): List<TrackedObject> = tracks.filter { it.isBall }.sortedByDescending { it.confidence }

    /** Lookup by track id (for target commitment in the COLLECT phase). */
    fun trackById(id: Int): TrackedObject? = tracks.firstOrNull { it.id == id }

    /** Forgets a ball (confirmed picked up) so it is never re-chased. */
    fun removeBall(id: Int) {
        tracks.removeAll { it.id == id && it.isBall }
    }

    /** Remembered flowers. */
    fun flowers(): List<TrackedObject> = tracks.filter { !it.isBall && it.label == "Flower" }

    /**
     * Flowers that may be pulled RIGHT NOW — i.e. not sitting in a depletion
     * cooldown. The planner only ever considers these.
     *
     * A flower is excluded when EITHER its own track record is in cooldown
     * OR it sits on a spot we already emptied (the spatial check catches the
     * case where the camera re-identified the flower as a brand-new track
     * and handed it a fresh id).
     */
    fun availableFlowers(nowNanos: Long): List<TrackedObject> =
        flowers().filter { flower ->
            flowerState(flower.id, nowNanos) == FlowerState.FULL &&
                !isDepletedSpot(flower.fieldXcm, flower.fieldYcm, nowNanos)
        }

    /** Current depletion state of a single flower. */
    fun flowerState(flowerId: Int, nowNanos: Long): FlowerState {
        val record = flowerDepletion[flowerId] ?: return FlowerState.FULL
        if (record.state == FlowerState.FULL) return FlowerState.FULL
        val since = (nowNanos - record.lastPulledNanos) / 1e9
        val required = FLOWER_COOLDOWN_BASE_SEC + record.pullCount * FLOWER_COOLDOWN_PER_PULL_SEC
        return when {
            since < required -> FlowerState.DEPLETED
            since < required + FLOWER_REGEN_SEC -> FlowerState.REGENERATING
            else -> FlowerState.FULL
        }
    }

    /** Seconds of cooldown still owed on a single flower. */
    fun flowerCooldownRemainingSec(flowerId: Int, nowNanos: Long): Double =
        flowerDepletion[flowerId]?.cooldownRemainingSec(nowNanos) ?: 0.0

    /** How many times a flower has been pulled. */
    fun flowerPullCount(flowerId: Int): Int = flowerDepletion[flowerId]?.pullCount ?: 0

    /** Active flower clusters (clustered, with depletion/cooldown state). */
    fun flowerClusters(): List<FlowerCluster> = flowerClusters
        .filter { it.state != FlowerState.UNKNOWN || it.pullCount == 0 }
        .sortedByDescending { it.totalNectarValue }

    /**
     * Flower clusters that are usable right now — FULL and containing at
     * least one flower that is not itself in cooldown (and not sitting on a
     * spot we already emptied).
     */
    fun availableFlowerClusters(nowNanos: Long): List<FlowerCluster> {
        val usableIds = availableFlowers(nowNanos).map { it.id }.toSet()
        return flowerClusters().filter { cluster ->
            cluster.state == FlowerState.FULL &&
                cluster.flowers.any { it.id in usableIds }
        }
    }

    /** Best flower cluster to visit (highest nectar value, not depleted, reachable in time). */
    fun bestFlowerCluster(
        robotXcm: Double,
        robotYcm: Double,
        timeRemainingSec: Double,
        ballsHeld: Int,
        nowNanos: Long = System.nanoTime(),
    ): FlowerCluster? {
        val hivePos = hive()
        val distToHive = hivePos?.let { hypot(it.first - robotXcm, it.second - robotYcm) } ?: 150.0
        val baseScoreTime = distToHive / ROBOT_SPEED_CM_PER_SEC
        val hiveObstaclePenalty = if (hivePos != null && isPathBlocked(robotXcm, robotYcm, hivePos.first, hivePos.second)) 3.0 else 0.0
        val scoreTime = baseScoreTime + PLANNER_SCORE_TOTAL_SEC + hiveObstaclePenalty
        val usableIds = availableFlowers(nowNanos).map { it.id }.toSet()
        return availableFlowerClusters(nowNanos).firstOrNull { cluster ->
            val usable = cluster.flowers.count { it.id in usableIds }
            usable > 0 &&
                ballsHeld + usable <= MAX_BALLS_HELD &&
                timeRemainingSec > cluster.distanceTo(robotXcm, robotYcm) / ROBOT_SPEED_CM_PER_SEC +
                    PLANNER_FLOWER_TOTAL_SEC * usable + scoreTime + PLANNER_ENDGAME_MARGIN_SEC
        }
    }

    /**
     * Marks a flower as pulled (depleted), triggering its cooldown.
     *
     * Three things happen:
     *
     *  1. the flower's OWN record enters cooldown (this survives re-clustering
     *     as long as the track id holds),
     *  2. a spatial [DepletedPatch] is recorded at the flower's field
     *     position — this is the one that survives the camera re-identifying
     *     the flower as a brand-new track,
     *  3. the cluster containing it is marked depleted so the planner stops
     *     steering at an empty patch.
     */
    fun markFlowerPulled(flowerId: Int, nowNanos: Long) {
        val record = flowerDepletion.getOrPut(flowerId) { FlowerDepletion(flowerId) }
        record.pullCount++
        record.lastPulledNanos = nowNanos
        record.state = FlowerState.DEPLETED

        // Remember WHERE this was, so the cooldown survives re-identification.
        val flower = tracks.firstOrNull { it.id == flowerId }
        if (flower != null) {
            markPatchDepleted(flower.fieldXcm, flower.fieldYcm, nowNanos)
        }

        // Find cluster containing this flower
        for (cluster in flowerClusters) {
            if (cluster.flowers.any { it.id == flowerId }) {
                cluster.state = FlowerState.DEPLETED
                cluster.lastPulledNanos = nowNanos
                cluster.pullCount++
                // The flower STAYS in the cluster's list: removing it would
                // change the cluster's identity and the rebuilt cluster would
                // lose the depletion flag. Its own state (DEPLETED) already
                // stops it contributing nectar value.
                break
            }
        }
        flowerClusterRevision++
    }

    /**
     * Records (or refreshes) a depleted spot on the field.
     *
     * Nearby patches are merged, so pulling three flowers from the same
     * flower bed produces ONE patch with pullCount = 3 — which is what makes
     * the cooldown grow with every pull (diminishing returns) instead of
     * being reset by each new pull.
     */
    private fun markPatchDepleted(xcm: Double, ycm: Double, nowNanos: Long) {
        for (patch in depletedPatches) {
            if (hypot(patch.xcm - xcm, patch.ycm - ycm) <= FLOWER_PATCH_MERGE_CM) {
                patch.pullCount++
                patch.lastPulledNanos = nowNanos
                return
            }
        }
        depletedPatches.add(DepletedPatch(xcm, ycm, 1, nowNanos))
    }

    /** The depleted spot nearest a field position, if any is on cooldown. */
    fun nearestDepletedPatch(
        xcm: Double,
        ycm: Double,
        nowNanos: Long,
    ): DepletedPatch? =
        depletedPatches
            .filter { it.onCooldown(nowNanos) }
            .minByOrNull { hypot(it.xcm - xcm, it.ycm - ycm) }

    /** True when a field position sits on a spot we already emptied. */
    fun isDepletedSpot(xcm: Double, ycm: Double, nowNanos: Long): Boolean {
        for (patch in depletedPatches) {
            if (!patch.onCooldown(nowNanos)) continue
            if (hypot(patch.xcm - xcm, patch.ycm - ycm) <= FLOWER_PATCH_MERGE_CM) return true
        }
        return false
    }

    /** Forces an immediate re-cluster (after a pull, or a big camera change). */
    fun forceClusterRebuild(nowNanos: Long) {
        rebuildFlowerClusters(nowNanos)
        lastClusterRebuildNanos = nowNanos
    }

    /**
     * The clustering epsilon, adapted to how the flowers are actually spread.
     *
     * A fixed epsilon is wrong at both ends of the density range: with a
     * scattered field it splits every flower into its own lonely cluster
     * (so the robot only ever plans for one nectar ball at a time), and with
     * a dense patch it merges two genuinely separate patches into one and
     * sends the robot to the wrong side of the field.
     *
     * The scale of a point set is well estimated by the mean distance to
     * each point's nearest neighbour, so:
     *
     *     eps = clamp(k * meanNearestNeighbourDistance, min, max)
     *
     * Sparse field -> big eps (group loosely spread flowers). Dense patch ->
     * small eps (keep separate patches separate).
     */
    fun adaptiveClusterEps(flowers: List<TrackedObject>): Double {
        if (flowers.size < 2) return FLOWER_CLUSTER_EPS_MIN_CM
        var sumNearest = 0.0
        for (f in flowers) {
            var nearest = Double.MAX_VALUE
            for (o in flowers) {
                if (o.id == f.id) continue
                val d = hypot(f.fieldXcm - o.fieldXcm, f.fieldYcm - o.fieldYcm)
                if (d < nearest) nearest = d
            }
            if (nearest < Double.MAX_VALUE) sumNearest += nearest
        }
        val meanNearest = sumNearest / flowers.size
        if (!meanNearest.isFinite()) return FLOWER_CLUSTER_EPS_MIN_CM
        return (FLOWER_CLUSTER_EPS_KNN_FACTOR * meanNearest)
            .coerceIn(FLOWER_CLUSTER_EPS_MIN_CM, FLOWER_CLUSTER_EPS_MAX_CM)
    }

    /**
     * Rebuilds flower clusters using adaptive-epsilon DBSCAN-like clustering
     * (minPts = 1). The epsilon is derived from the current spread of the
     * visible flowers, so the same code handles a sparse field and a dense
     * flower bed.
     */
    private fun rebuildFlowerClusters(nowNanos: Long) {
        val allFlowers = flowers()
        if (allFlowers.isEmpty()) {
            if (flowerClusters.isNotEmpty()) {
                flowerClusters.clear()
                flowerClusterRevision++
            }
            return
        }

        val eps = adaptiveClusterEps(allFlowers)
        val visited = mutableSetOf<Int>()
        val newClusters = mutableListOf<FlowerCluster>()

        for (flower in allFlowers) {
            if (flower.id in visited) continue
            visited.add(flower.id)

            // Find all flowers within eps
            val clusterFlowers = mutableListOf(flower)
            for (other in allFlowers) {
                if (other.id in visited) continue
                if (flower.distanceTo(other.fieldXcm, other.fieldYcm) <= eps) {
                    clusterFlowers.add(other)
                    visited.add(other.id)
                }
            }

            // Compute cluster center and total nectar value. Only flowers
            // that are actually usable contribute nectar value.
            val centerX = clusterFlowers.map { it.fieldXcm }.average()
            val centerY = clusterFlowers.map { it.fieldYcm }.average()
            val usable = clusterFlowers.count { flowerState(it.id, nowNanos) == FlowerState.FULL }
            val totalNectar = usable.toDouble() * NECTAR_VALUE

            // Cluster state is DERIVED, not remembered: the cluster objects
            // are thrown away every rebuild, so anything stored on them is
            // lost. The truth lives in the per-flower records and in the
            // spatial depleted patches.
            val derived = deriveClusterState(clusterFlowers, centerX, centerY, nowNanos)

            val cluster = FlowerCluster(
                id = nextClusterId++,
                flowers = clusterFlowers,
                centerXcm = centerX,
                centerYcm = centerY,
                totalNectarValue = totalNectar,
                state = derived.state,
                lastPulledNanos = derived.lastPulledNanos,
                pullCount = derived.pullCount,
            )
            newClusters.add(cluster)
        }

        // Did the cluster set actually change? Only then bump the revision,
        // so consumers can watch it as a cheap "something changed" signal.
        val changed = newClusters.size != flowerClusters.size ||
            newClusters.indices.any { i ->
                newClusters[i].flowers.map { it.id }.toSet() !=
                    flowerClusters[i].flowers.map { it.id }.toSet()
            }
        flowerClusters.clear()
        flowerClusters.addAll(newClusters)
        if (changed) flowerClusterRevision++
    }

    /** Advances every flower's cooldown/regrowth state machine. */
    private fun updateFlowerDepletionStates(nowNanos: Long) {
        for (record in flowerDepletion.values) {
            if (record.state == FlowerState.FULL) continue
            val since = (nowNanos - record.lastPulledNanos) / 1e9
            val required = FLOWER_COOLDOWN_BASE_SEC + record.pullCount * FLOWER_COOLDOWN_PER_PULL_SEC
            record.state = when {
                since < required -> FlowerState.DEPLETED
                since < required + FLOWER_REGEN_SEC -> FlowerState.REGENERATING
                else -> FlowerState.FULL
            }
        }
    }

    /**
     * The cluster state implied by the per-flower records and the spatial
     * depleted patches.
     *
     * A patch we emptied that is still on cooldown puts the WHOLE cluster on
     * cooldown: the nectar lives in one physical flower bed, so driving back
     * to that bed is pointless until it regrows.
     */
    private fun deriveClusterState(
        clusterFlowers: List<TrackedObject>,
        centerXcm: Double,
        centerYcm: Double,
        nowNanos: Long,
    ): ClusterStateView {
        val patch = depletedPatches
            .filter { it.onCooldown(nowNanos) }
            .minByOrNull { hypot(it.xcm - centerXcm, it.ycm - centerYcm) }
        val onDepletedPatch = patch != null &&
            hypot(patch.xcm - centerXcm, patch.ycm - centerYcm) <= FLOWER_PATCH_MERGE_CM

        val usable = clusterFlowers.count { flowerState(it.id, nowNanos) == FlowerState.FULL }
        val state = when {
            onDepletedPatch -> FlowerState.DEPLETED
            usable > 0 -> FlowerState.FULL
            clusterFlowers.any { flowerState(it.id, nowNanos) == FlowerState.REGENERATING } ->
                FlowerState.REGENERATING
            clusterFlowers.any { flowerState(it.id, nowNanos) == FlowerState.DEPLETED } ->
                FlowerState.DEPLETED
            else -> FlowerState.UNKNOWN
        }
        return ClusterStateView(
            state = state,
            lastPulledNanos = if (onDepletedPatch) patch!!.lastPulledNanos else 0L,
            pullCount = if (onDepletedPatch) patch!!.pullCount else 0,
        )
    }

    /** The three numbers a cluster's state is made of. */
    private data class ClusterStateView(
        val state: FlowerState,
        val lastPulledNanos: Long,
        val pullCount: Int,
    )

    /** Updates flower cluster states (depletion -> regenerating -> full). */
    private fun updateFlowerClusterStates(nowNanos: Long) {
        for (cluster in flowerClusters) {
            val derived = deriveClusterState(
                cluster.flowers, cluster.centerXcm, cluster.centerYcm, nowNanos,
            )
            cluster.state = derived.state
            if (derived.pullCount > 0) {
                cluster.lastPulledNanos = derived.lastPulledNanos
                cluster.pullCount = derived.pullCount
            }
        }
    }

    /** Remembered opponent robots, freshest first. */
    fun opponents(): List<TrackedObject> =
        tracks.filter { !it.isBall && it.label != "Flower" }.sortedByDescending { it.confidence }

    /** Best remembered hive position (field cm), or null when never seen. */
    fun hive(): Pair<Double, Double>? =
        if (hiveConfidence > 0.1) hiveXcm to hiveYcm else null

    fun hiveConfidence(): Double = hiveConfidence

    fun clear() {
        tracks.clear()
        flowerClusters.clear()
        fieldObstacles.clear()
        flowerDepletion.clear()
        depletedPatches.clear()
        opponentParticles.clear()
        hiveConfidence = 0.0
        hiveSeenNanos = 0L
        lastUpdateNanos = 0L
        lastClusterRebuildNanos = 0L
        flowerClusterRevision++
        initFieldObstacles()
    }

    /** Initializes static field obstacles (hive posts, perimeter walls). */
    private fun initFieldObstacles() {
        // Perimeter walls (field boundaries: 360cm x 240cm standard)
        val wallThickness = 5.0
        val fieldWidth = 360.0
        val fieldHeight = 240.0
        
        fieldObstacles.addAll(listOf(
            // Left wall
            FieldObstacle(1, -fieldWidth/2.0, 0.0, wallThickness, "Left Wall"),
            // Right wall
            FieldObstacle(2, fieldWidth/2.0, 0.0, wallThickness, "Right Wall"),
            // Top wall
            FieldObstacle(3, 0.0, fieldHeight/2.0, wallThickness, "Top Wall"),
            // Bottom wall
            FieldObstacle(4, 0.0, -fieldHeight/2.0, wallThickness, "Bottom Wall"),
        ))
    }

    /** Updates hive post positions when hive is detected. */
    fun updateHivePosts(hiveX: Double, hiveY: Double, hiveHeading: Double) {
        // Remove old hive posts
        fieldObstacles.removeAll { it.isHivePost }
        
        val postRadius = 5.0
        val offset = 30.0
        
        // Four posts at hive corners (rotated by hive heading)
        val corners = listOf<Pair<Double, Double>>(
            offset to offset,
            -offset to offset,
            -offset to -offset,
            offset to -offset,
        )
        
        for ((i, corner) in corners.withIndex()) {
            val (cx, cy) = corner
            val cosH = cos(hiveHeading)
            val sinH = sin(hiveHeading)
            val rx = cx * cosH - cy * sinH
            val ry = cx * sinH + cy * cosH
            fieldObstacles.add(FieldObstacle(
                100 + i, // IDs 100-103 for hive posts
                hiveX + rx,
                hiveY + ry,
                postRadius,
                "Hive Post ${i+1}",
                isHivePost = true
            ))
        }
    }

    /** Returns all field obstacles. */
    fun fieldObstacles(): List<FieldObstacle> = fieldObstacles

    /** Checks if a straight-line path is blocked by any field obstacle. */
    fun isPathBlocked(fromX: Double, fromY: Double, toX: Double, toY: Double, robotRadiusCm: Double = 25.0): Boolean {
        return fieldObstacles.any { it.blocksPath(fromX, fromY, toX, toY, robotRadiusCm) }
    }

    /** Finds a clear path around obstacles (simple waypoint offset). */
    fun findClearPath(fromX: Double, fromY: Double, toX: Double, toY: Double, robotRadiusCm: Double = 25.0): Pair<Double, Double>? {
        if (!isPathBlocked(fromX, fromY, toX, toY, robotRadiusCm)) return null
        
        // Try offsetting perpendicular to the path
        val dx = toX - fromX
        val dy = toY - fromY
        val dist = hypot(dx, dy)
        if (dist < 1.0) return null
        
        val ux = -dy / dist // perpendicular unit vector
        val uy = dx / dist
        
        // Try both sides
        for (sign in listOf(1.0, -1.0)) {
            val offsetDist = 40.0 * sign // 40cm offset
            val midX = (fromX + toX) / 2.0 + ux * offsetDist
            val midY = (fromY + toY) / 2.0 + uy * offsetDist
            
            if (!isPathBlocked(fromX, fromY, midX, midY, robotRadiusCm) &&
                !isPathBlocked(midX, midY, toX, toY, robotRadiusCm)) {
                return midX to midY
            }
        }
        return null
    }

    /** Compact live view for the Driver Station. */
    fun summary(): String {
        val oppSummary = opponents().map { "${it.label[0]}=${it.intent.name[0]}${if (it.intentConfidence > 0.5) "✓" else ""}" }.joinToString(",")
        return "balls:%d flowers:%d opp:%d[%s] hive:%s".format(
            balls().size, flowers().size, opponents().size, oppSummary,
            if (hive() != null) "yes(%.0f%%)".format(hiveConfidence * 100) else "no",
        )
    }

    /**
     * Serializable snapshot of the whole model — the Black-Box recorder (the
     * superhuman branch) logs this ~10 Hz; core master never calls it.
     */
    fun logPayload(): Map<String, Any?> = mapOf(
        "balls" to balls().map {
            mapOf("id" to it.id, "x" to it.fieldXcm, "y" to it.fieldYcm,
                  "lbl" to it.label, "conf" to it.confidence)
        },
        "opp" to opponents().map {
            mapOf("id" to it.id, "x" to it.fieldXcm, "y" to it.fieldYcm,
                  "conf" to it.confidence, "vx" to it.velXcmPerSec, "vy" to it.velYcmPerSec,
                  "intent" to it.intent.name, "intentConf" to it.intentConfidence)
        },
        "flowers" to flowers().map {
            mapOf("id" to it.id, "x" to it.fieldXcm, "y" to it.fieldYcm, "conf" to it.confidence)
        },
        "hiveX" to (if (hive() != null) hiveXcm else null),
        "hiveY" to (if (hive() != null) hiveYcm else null),
        "hiveConf" to hiveConfidence,
    )

    /**
     * Chase utility of a ball track (same shape as the old BallLocator
     * utility, confidence-weighted; live sightings beat stale ones).
     */
    fun chaseUtilityOf(track: TrackedObject, robotXcm: Double, robotYcm: Double, nowNanos: Long): Double {
        val distCm = track.distanceTo(robotXcm, robotYcm)
        val visibility = if (track.ageSec(nowNanos) <= WM_VISIBLE_FRESH_SEC) 1.0 else 0.75
        return track.value * track.confidence * PRIORITY_VALUE_WEIGHT *
            visibility / (1.0 + (distCm / 100.0) * PRIORITY_DISTANCE_WEIGHT)
    }

    /**
     * Capacity-aware chase pick for the COLLECT phase (the core target
     * selector; the superhuman branch's StrategicPlanner overrides this
     * with lookahead + opponent discounting):
     *  - fresh-enough sighting of a solid-enough blob only,
     *  - exactly one slot left -> highest-value ball outright when
     *    LAST_SLOT_PREFER_HIGHEST_VALUE is set, else highest utility.
     */
    fun bestBallToChase(robotXcm: Double, robotYcm: Double, remainingSlots: Int, nowNanos: Long): TrackedObject? {
        if (remainingSlots <= 0) return null
        val candidates = balls().filter {
            it.ageSec(nowNanos) <= DEAD_RECKON_MAX_AGE_SEC && it.lastAreaPx >= CHASE_MIN_CONTOUR_AREA_PX
        }
        if (candidates.isEmpty()) return null
        return if (remainingSlots == 1 && LAST_SLOT_PREFER_HIGHEST_VALUE) {
            candidates.maxWithOrNull(
                compareBy<TrackedObject> { it.value }.thenBy { it.distanceTo(robotXcm, robotYcm) },
            )
        } else {
            candidates.maxByOrNull { chaseUtilityOf(it, robotXcm, robotYcm, nowNanos) }
        }
    }
}

// ============================================================================
// SECTION 5: StrategicPlanner (was StrategicPlanner.kt) — SUPERHUMAN-ONLY
// ============================================================================


/**
 * ============ STRATEGIC PLANNER — the lookahead brain ============
 *
 * Replaces the reactive "what do I see RIGHT NOW" strategy enum with a
 * value/cost search over the WorldModel's memory:
 *
 *  * every candidate action (collect ball i / go score / use flower / park)
 *    gets an expected value and an estimated time cost,
 *  * balls an opponent is closer to get discounted (they'll take them),
 *  * the ENDGAME is modeled: nothing starts that can't pay back before the
 *    clock runs out — full payloads force a score run, the last seconds
 *    force park.
 *
 * Deliberately small and deterministic (runs in microseconds) — the structure
 * is ready for a deeper search (MCTS) later, but this level already beats the
 * greedy controller because it remembers unseen balls and budgets time.
 *
 * It also powers SHADOW MODE: called during human teleop it logs what it
 * would do (without acting) — that log is how we'll prove it outplays the
 * human before ever letting it drive.
 */
enum class PlannerAction { COLLECT_BALL, GO_SCORE, USE_FLOWER, PARK, HOLD }

data class PlannerDecision(
    val action: PlannerAction,
    val targetId: Int,
    val targetLabel: String,
    val targetFieldXcm: Double,
    val targetFieldYcm: Double,
    val expectedValue: Double,
    val estCostSec: Double,
    val rationale: String,
)
