 package org.firstinspires.ftc.teamcode

import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.vision.CameraId
import org.firstinspires.ftc.teamcode.vision.VisionConfig
import org.firstinspires.ftc.vision.VisionProcessor
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor
import org.opencv.core.Mat
import org.opencv.core.Point
import java.util.Locale
import java.util.concurrent.atomic.AtomicReference
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sqrt

/**
 * ============ HIVE SCORING ASSIST (known-target pathway) ============
 *
 * The KNOWN-TARGET side of the vision stack: real distance and angle to hive
 * AprilTags, used by BOTH autonomous (drive into the shooting window, then
 * fire) and teleop (alignment assists). The generic feature tracker in the
 * `vision` package is a separate pathway and never depends on this.
 *
 * HOW TO ADJUST WHERE THE ROBOT SHOOTS FROM: edit [HiveScoringConfig].
 * The live tag distance/angle is visible in telemetry AND drawn on the
 * Driver Station camera preview by [TagInfoOverlay].
 */

/**
 * ==== ADJUST THESE to tune the scoring window ====
 * The robot only fires when the tag is inside the distance window AND the
 * bearing is inside the angle limit. The same window drives autonomous and
 * the teleop alignment assist, so there is exactly one place to tune.
 */
object HiveScoringConfig {
    // -- The shooting range window --
    // Shooting is DISTANCE-BASED now: once the RPM-vs-distance curve is
    // calibrated (globals.kt SHOOTER_CALIBRATION_*), the max range comes from
    // the curve itself (ShooterModel.effectiveMaxRangeCm()) so the robot can
    // fire from anywhere it can physically reach. SHOOT_MAX_RANGE_CM is only
    // the fallback until you calibrate. SHOOT_MIN stays a hard safety floor.
    const val SHOOT_MIN_RANGE_CM = 60.0   // closer than this = TOO_CLOSE (no fire)
    const val SHOOT_MAX_RANGE_CM = 140.0  // fallback only — the calibration curve overrides
    const val SHOOT_MAX_BEARING_DEG = 10.0 // must aim within this angle of the tag

    // -- Alignment controller gains (power per unit error) --
    const val RANGE_CENTER_CM = (SHOOT_MIN_RANGE_CM + SHOOT_MAX_RANGE_CM) / 2.0
    const val RANGE_GAIN = 0.004          // forward power per cm of range error
    const val BEARING_GAIN = 0.02         // turn power per degree of bearing error
    const val ASSIST_MAX_POWER = 0.30     // cap on assist/auto-align drive powers

    // -- Autonomous behavior --
    const val SEARCH_TURN_POWER = 0.15    // spin-in-place sweep when no tag visible
    const val SEARCH_TIMEOUT_MS = 5000.0
    const val ALIGN_TIMEOUT_MS = 8000.0
    const val FIRE_TIME_MS = 2500.0        // flywheel run time to launch held balls

    // -- Ball capture assist --
    const val CAPTURE_APPROACH_CM = 18.0   // standoff the assist holds from a ball
    const val CAPTURE_RANGE_GAIN = 0.02    // forward power per cm
    const val CAPTURE_BEARING_GAIN = 0.02  // strafe power per degree
    const val MIN_BLOB_AREA_PX = 150.0     // blob noise floor (px^2 @640x480)
}

/** Where the robot currently sits relative to the shooting window. */
enum class ScoringZone { NO_TAG, TOO_CLOSE, IN_RANGE, TOO_FAR, ALIGNED }

/**
 * One hive target measurement (immutable snapshot). Official BioBuzz hives are
 * AprilTag CLUSTERS (SDK 12.0), so a target is usually a cluster; the legacy
 * single-tag path remains as a fallback for uncalibrated/custom tag setups.
 */
data class HiveTarget(
    /** Single-tag ID, or -1 when this target is a hive CLUSTER. */
    val tagId: Int,
    /** "RED SCORING" etc. for clusters, "Tag 34" for the single-tag fallback. */
    val label: String,
    /** Straight-line distance to the aim point (tag or cluster origin), cm. */
    val distanceCm: Double,
    /** + = target is right of the camera center, degrees. */
    val bearingDeg: Double,
    val screenX: Double,
    val screenY: Double,
    /** Single tags only: tag diagonal in pixels — bigger = closer/cleaner. */
    val pixelDiagonal: Double,
    /** Clusters only: percent of the cluster's tags currently visible. */
    val percentClusterFound: Int,
    /** True when the full 6-DOF pose was available (tag library + calibration). */
    val fromFtcPose: Boolean,
    val timestampNanos: Long,
)

/**
 * Reads hive AprilTag detections and turns them into range/bearing.
 *
 * Targeting paths, best available wins:
 *  1. CLUSTERS (preferred, SDK 12.0): official BioBuzz hives are 4-tag
 *     clusters whose origin is the CENTER OF THE CELL OPENING, so the cluster
 *     ftcPose points directly at the scoring aim point. Requires the game tag
 *     library (default) AND a calibrated camera (teamwebcamcalibrations.xml).
 *  2. Single tags + [AprilTagDetection.ftcPose] — full pose, needs single-tag
 *     library metadata AND a calibrated camera.
 *  3. Pixel-geometry fallback for single tags — the tag size is known
 *     (APRILTAG_PHYSICAL_WIDTH_MM), so distance = focal * width / pixels and
 *     bearing = atan(pixel offset / focal). Works with no library metadata;
 *     only fires when tags are reported as singles (custom tag setups).
 */
class HiveScoring(
    private val aprilTagProcessor: AprilTagProcessor,
    /** Which camera the tags actually run on — in single-camera mode the
     *  FRONT camera carries them and its intrinsics must be used. */
    cameraId: CameraId = CameraId.UP,
) {

    private val upIntrinsics = VisionConfig.calibrationFor(cameraId).intrinsics

    /** When false, update() returns null immediately without querying the AprilTagProcessor.
     *  Set to true only during ALIGN phase (and INIT for alliance selection). */
    var aprilTagEnabled = true

    /**
     * Latest hive target for the SELECTED ALLIANCE ([SELECTED_TEAM], picked
     * during INIT in both match OpModes) — best visible own-hive cluster, else
     * best own single tag, or null. Opposing hive tags/clusters are ignored, so
     * the robot can never line up on (or fire into) the wrong hive.
     */
    fun update(): HiveTarget? {
        if (!aprilTagEnabled) return null
        val team = SELECTED_TEAM
        val ourClusters = teamClusterNames(team)
        val ourIds = teamTagIds(team).toHashSet()

        // The detections list is owned by the vision thread — snapshot defensively.
        val tags = try {
            aprilTagProcessor.detections.toList()
        } catch (_: Exception) {
            emptyList()
        }

        // SDK 12.0: hives arrive as AprilTagClusterDetection. Clusters are the
        // preferred target — their origin is the Cell-opening center, the exact
        // scoring aim point. Single tags remain a fallback path.
        var bestCluster: HiveTarget? = null
        var bestSingle: HiveTarget? = null
        for (tag in tags) {
            when (tag) {
                is AprilTagClusterDetection -> {
                    if (tag.metadata?.shortName !in ourClusters) continue
                    val target = clusterToTarget(tag) ?: continue
                    if (bestCluster == null ||
                        target.percentClusterFound > bestCluster.percentClusterFound
                    ) bestCluster = target
                }
                is AprilTagSingleDetection -> {
                    if (tag.id !in ourIds) continue
                    val target = toTarget(tag) ?: continue
                    if (bestSingle == null ||
                        target.pixelDiagonal > bestSingle.pixelDiagonal
                    ) bestSingle = target
                }
            }
        }
        return bestCluster ?: bestSingle
    }

    /**
     * CLUSTER path (SDK 12.0, preferred): the pose is relative to the cluster
     * origin — the center of the Cell opening — so range/bearing are the true
     * scoring geometry. NOTE: clusters expose no pixel corners/center, so this
     * path requires the ftcPose (calibrated camera); there is no pixel fallback.
     */
    private fun clusterToTarget(cluster: AprilTagClusterDetection): HiveTarget? {
        val ftc = cluster.ftcPose ?: return null
        val name = cluster.metadata?.shortName ?: return null
        // SDK sign convention: ftc.bearing is POSITIVE when the tag is to the
        // LEFT (positive yaw = CCW to face it). HiveTarget normalizes to
        // "+ = right of center" so all steering/overlay/world-model math can
        // use one convention — negate here. (Do NOT remove: every consumer —
        // rangeCorrection's turn, the overlay, WorldModel — assumes + = right.)
        val bearingDeg = -ftc.bearing.toDouble()
        // Clusters do not report a screen position — reconstruct an approximate
        // one from the bearing so the preview overlay lands near the hive.
        val screenX = upIntrinsics.centerX +
            upIntrinsics.focalLengthPx * Math.tan(Math.toRadians(bearingDeg))
        return HiveTarget(
            tagId = -1,
            label = name,
            distanceCm = ftc.range * 2.54,
            bearingDeg = bearingDeg,
            screenX = screenX,
            screenY = upIntrinsics.centerY,
            pixelDiagonal = 0.0,
            percentClusterFound = cluster.percentClusterFound,
            fromFtcPose = true,
            timestampNanos = cluster.frameAcquisitionNanoTime,
        )
    }

    private fun toTarget(tag: AprilTagSingleDetection): HiveTarget? {
        val center = tag.center ?: return null
        val diag = pixelDiagonalOf(tag.corners)

        val ftc = tag.ftcPose
        if (ftc != null) {
            return HiveTarget(
                tagId = tag.id,
                label = "Tag ${tag.id}",
                distanceCm = ftc.range * 2.54,
                // ftc.bearing: + = tag LEFT (SDK) -> negate for "+ = right"
                bearingDeg = -ftc.bearing.toDouble(),
                screenX = center.x,
                screenY = center.y,
                pixelDiagonal = diag,
                percentClusterFound = 0,
                fromFtcPose = true,
                timestampNanos = tag.frameAcquisitionNanoTime,
            )
        }

        // Pixel fallback: needs a visible corner quad.
        if (diag <= 1.0) return null
        val sidePx = diag / sqrt(2.0)
        val distanceCm = upIntrinsics.focalLengthPx * APRILTAG_PHYSICAL_WIDTH_MM / sidePx / 10.0
        val bearingDeg = Math.toDegrees(
            atan2(center.x - upIntrinsics.centerX, upIntrinsics.focalLengthPx),
        )
        return HiveTarget(
            tagId = tag.id,
            label = "Tag ${tag.id}",
            distanceCm = distanceCm,
            bearingDeg = bearingDeg,
            screenX = center.x,
            screenY = center.y,
            pixelDiagonal = diag,
            percentClusterFound = 0,
            fromFtcPose = false,
            timestampNanos = tag.frameAcquisitionNanoTime,
        )
    }

    /** Max pairwise corner distance = the tag's bounding diagonal, px. */
    private fun pixelDiagonalOf(corners: Array<Point>?): Double {
        if (corners == null || corners.size < 4) return 0.0
        var best = 0.0
        for (i in corners.indices) {
            for (j in i + 1 until corners.size) {
                best = max(best, hypot(corners[i].x - corners[j].x, corners[i].y - corners[j].y))
            }
        }
        return best
    }

    /** Classifies a target against the shooting window (calibration-aware). */
    fun zoneOf(t: HiveTarget?): ScoringZone {
        if (t == null) return ScoringZone.NO_TAG
        val bearingOk = abs(t.bearingDeg) <= HiveScoringConfig.SHOOT_MAX_BEARING_DEG
        return when {
            t.distanceCm < HiveScoringConfig.SHOOT_MIN_RANGE_CM -> ScoringZone.TOO_CLOSE
            t.distanceCm > ShooterModel.effectiveMaxRangeCm() -> ScoringZone.TOO_FAR
            !bearingOk -> ScoringZone.IN_RANGE // distance OK, still turning to aim
            else -> ScoringZone.ALIGNED
        }
    }

    /**
     * Flywheel RPM to land a ball on the target from wherever we stand: the
     * calibrated curve ([ShooterModel]) evaluated at the LIVE tag distance.
     * Uncalibrated curve or no target -> full speed (the old behavior).
     */
    fun shootingRpmFor(t: HiveTarget?): Double =
        if (t == null) FLYWHEEL_MOTOR_MAX_RPM
        else ShooterModel.rpmForDistance(t.distanceCm)

    /**
     * Bounded (forward, turn) powers steering toward the window center while
     * zeroing the bearing. Turn follows the Road Runner convention (CCW
     * positive), so the same pair drives RR in autonomous and the NextFTC
     * drivetrain in teleop. Returns (0, 0) with no target.
     */
    fun rangeCorrection(t: HiveTarget?): Pair<Double, Double> {
        if (t == null) return 0.0 to 0.0
        val cfg = HiveScoringConfig
        val rangeErr = t.distanceCm - cfg.RANGE_CENTER_CM // + = too far -> drive forward
        // Don't charge toward a tag that's far off-axis; aim first.
        val forward = (cfg.RANGE_GAIN * rangeErr * cos(Math.toRadians(t.bearingDeg)))
            .coerceIn(-cfg.ASSIST_MAX_POWER, cfg.ASSIST_MAX_POWER)
        val turn = (-cfg.BEARING_GAIN * t.bearingDeg) // +bearing (tag right) -> turn CW
            .coerceIn(-cfg.ASSIST_MAX_POWER, cfg.ASSIST_MAX_POWER)
        return forward to turn
    }

    /** Live view of distance/angle/zone for the Driver Station telemetry. */
    fun addTelemetry(t: Telemetry, target: HiveTarget?) {
        val cfg = HiveScoringConfig
        val zone = zoneOf(target)
        t.addData("TEAM", SELECTED_TEAM.name)
        t.addData(
            "HIVE window",
            "${cfg.SHOOT_MIN_RANGE_CM.toInt()}-${ShooterModel.effectiveMaxRangeCm().toInt()} cm " +
                "(${ShooterModel.summary()}), |angle| <= ${cfg.SHOOT_MAX_BEARING_DEG.toInt()} deg",
        )
        if (target == null) {
            t.addData("HIVE tag", "none visible")
        } else {
            val quality = if (target.percentClusterFound > 0) {
                String.format(Locale.US, "%d%% cluster", target.percentClusterFound)
            } else {
                String.format(Locale.US, "%.0f px", target.pixelDiagonal)
            }
            t.addData(
                "HIVE ${target.label}",
                String.format(Locale.US, "%.1f cm, %.1f deg, %s", target.distanceCm, target.bearingDeg, quality) +
                    if (target.fromFtcPose) " [ftcPose]" else " [pixel]",
            )
        }
        t.addData("HIVE zone", zone)
    }
}

/** One ball detection from the FRONT camera blob locators (immutable). */
data class BallTarget(
    val label: String,
    /** Straight-line distance estimate, cm (from known physical ball width). */
    val distanceCm: Double,
    /** + = ball is right of the camera center, degrees. */
    val bearingDeg: Double,
    val screenX: Double,
    val screenY: Double,
    /** Contour area in px^2 — used as a confidence/noise floor. */
    val contourArea: Double,
)

/**
 * Wraps the pollen/nectar/flower ColorBlobLocatorProcessors so a single
 * instance feeds the FRONT portal AND provides targeting queries:
 *  - [nearestBall]      — teleop capture assist target (closest floor ball)
 *  - [bestBall]         — capacity-aware PRIORITY target for autonomous
 *                         (see the priority constants in globals.kt)
 *  - [nearestFlower]    — flower target for the customizable pull-out phase
 *
 * Distance/angle use the same pinhole math as the old VisionSystem
 * (C270_FOCAL_LENGTH_PIXELS at 640x480 + known physical widths from globals).
 */
class BallLocator {

    private data class Source(
        val proc: ColorBlobLocatorProcessor,
        val label: String,
        val widthMm: Double,
        val isBall: Boolean,
    )

    private val pollen = source(POLLEN_HSV_RANGE, "Pollen", POLLEN_PHYSICAL_WIDTH_MM, isBall = true)
    private val nectarBlue = source(NECTAR_BLUE_HSV_RANGE, "Nectar Blue", NECTAR_PHYSICAL_WIDTH_MM, isBall = true)
    private val nectarRed = source(NECTAR_RED_HSV_RANGE, "Nectar Red", NECTAR_PHYSICAL_WIDTH_MM, isBall = true)
    // Second half of the wrapping red hue band (H 168-180) — same label so it
    // is indistinguishable from the primary red source downstream.
    private val nectarRedWrap = source(NECTAR_RED_WRAP_HSV_RANGE, "Nectar Red", NECTAR_PHYSICAL_WIDTH_MM, isBall = true)
    private val flower = source(FLOWER_GREEN_HSV_RANGE, "Flower", FLOWER_PHYSICAL_WIDTH_MM, isBall = false)

    /**
     * Opponent robots: large dark blobs (same idea as VisionSystem's
     * robotDetector) — feeds the WorldModel's opponent tracking so the
     * StrategicPlanner can discount balls they'll reach first.
     */
    private val robotDetector = ColorBlobLocatorProcessor.Builder()
        .setTargetColorRange(ROBOT_DARK_HSV_RANGE)
        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
        .setDrawContours(false)
        .build()

    /** Add these to the FRONT camera portal. */
    val processors = listOf(
        pollen.proc, nectarBlue.proc, nectarRed.proc, nectarRedWrap.proc,
        flower.proc, robotDetector,
    )

    private fun source(
        range: org.firstinspires.ftc.vision.opencv.ColorRange,
        label: String,
        widthMm: Double,
        isBall: Boolean,
    ) = Source(
        ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(range)
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
            .setDrawContours(true)
            .build(),
        label, widthMm, isBall,
    )

    /** All currently visible, non-noise targets (floor balls + flowers),
     *  passed through the NeuralReferee per NN_DETECTOR_MODE (OFF/ADVISORY
     *  pass straight through; REFEREE removes uncorroborated blobs; PRIMARY
     *  can also add detector-only finds). */
    fun visibleTargets(): List<BallTarget> {
        val out = mutableListOf<BallTarget>()
        // nectarRedWrap must be listed too: it covers the wrapping half of the
        // red hue band (H 168-180) and shares the "Nectar Red" label — without
        // it those blobs are processed by the portal but never surfaced.
        for (src in listOf(pollen, nectarBlue, nectarRed, nectarRedWrap, flower)) {
            val blobs = try {
                src.proc.blobs.toList()
            } catch (_: Exception) {
                emptyList()
            }
            for (blob in blobs) {
                if (blob.contourArea.toDouble() < HiveScoringConfig.MIN_BLOB_AREA_PX) continue
                val box = blob.boxFit
                val pixelWidth = max(box.size.width, box.size.height)
                if (pixelWidth < 2.0) continue

                out.add(
                    BallTarget(
                        label = src.label,
                        distanceCm = C270_FOCAL_LENGTH_PIXELS * src.widthMm / pixelWidth / 10.0,
                        bearingDeg = Math.toDegrees(
                            atan2(box.center.x - GROUND_CAM_WIDTH / 2.0, C270_FOCAL_LENGTH_PIXELS),
                        ),
                        screenX = box.center.x,
                        screenY = box.center.y,
                        contourArea = blob.contourArea.toDouble(),
                    ),
                )
            }
        }
        return NeuralReferee.filter(out)
    }

    private fun balls(): List<BallTarget> = visibleTargets().filter { it.label != "Flower" }

    /** Closest floor ball across all colors (teleop capture assist target). */
    fun nearestBall(): BallTarget? = balls().minByOrNull { it.distanceCm }

    /** Closest flower, for the customizable pull-out phase. */
    fun nearestFlower(): BallTarget? = visibleTargets()
        .filter { it.label == "Flower" }
        .minByOrNull { it.distanceCm }

    /**
     * Opponent robots visible to the FRONT camera (large dark blobs that pass
     * the MIN_ROBOT_CONTOUR_AREA floor). Distance uses the same pinhole math
     * with the known robot width.
     */
    fun visibleRobots(): List<BallTarget> {
        val out = mutableListOf<BallTarget>()
        val blobs = try {
            robotDetector.blobs.toList()
        } catch (_: Exception) {
            emptyList()
        }
        for (blob in blobs) {
            if (blob.contourArea.toDouble() < MIN_ROBOT_CONTOUR_AREA) continue
            val box = blob.boxFit
            val pixelWidth = max(box.size.width, box.size.height)
            if (pixelWidth < 2.0) continue
            out.add(
                BallTarget(
                    label = "Robot",
                    distanceCm = C270_FOCAL_LENGTH_PIXELS * ROBOT_PHYSICAL_WIDTH_MM / pixelWidth / 10.0,
                    bearingDeg = Math.toDegrees(
                        atan2(box.center.x - GROUND_CAM_WIDTH / 2.0, C270_FOCAL_LENGTH_PIXELS),
                    ),
                    screenX = box.center.x,
                    screenY = box.center.y,
                    contourArea = blob.contourArea.toDouble(),
                ),
            )
        }
        return out
    }

    /**
     * Everything the FRONT camera sees as [Sighting]s for the [WorldModel]:
     * balls, flowers and opponent robots in one robot-relative list.
     */
    fun sightings(): List<Sighting> =
        visibleTargets().map {
            Sighting(it.label, it.distanceCm, it.bearingDeg, it.contourArea, isBall = it.label != "Flower")
        } + visibleRobots().map {
            Sighting("Robot", it.distanceCm, it.bearingDeg, it.contourArea, isBall = false)
        }

    /** Point value of a target (globals.kt: POLLEN_VALUE / NECTAR_VALUE). */
    fun ballValueOf(t: BallTarget): Double = when (t.label) {
        "Nectar Blue", "Nectar Red" -> NECTAR_VALUE
        else -> POLLEN_VALUE
    }

    /** Normal-slot utility: value density that balances worth against travel. */
    fun utilityOf(t: BallTarget): Double =
        ballValueOf(t) * PRIORITY_VALUE_WEIGHT /
            (1.0 + (t.distanceCm / 100.0) * PRIORITY_DISTANCE_WEIGHT)

    /**
     * Capacity-aware priority choice for autonomous:
     *  - remainingSlots <= 0      -> null (full, stop collecting)
     *  - exactly one slot left    -> highest-value ball outright (configurable)
     *  - otherwise                -> highest utility (value vs distance)
     */
    fun bestBall(remainingSlots: Int): BallTarget? {
        if (remainingSlots <= 0) return null
        val candidates = balls()
        if (candidates.isEmpty()) return null

        return if (remainingSlots == 1 && LAST_SLOT_PREFER_HIGHEST_VALUE) {
            candidates.maxWithOrNull(
                compareBy<BallTarget> { ballValueOf(it) }.thenBy { it.distanceCm },
            )
        } else {
            candidates.maxByOrNull { utilityOf(it) }
        }
    }
}

/**
 * Draws the live hive measurement ("34: 118cm  -6.2°  ALIGNED") next to the
 * tag on the Driver Station preview — this is how you SEE how far and at
 * what angle the robot is from the AprilTags while driving/aligning.
 * The target snapshot is published from the OpMode loop (thread-safe).
 */
class TagInfoOverlay : VisionProcessor {

    private val targetRef = AtomicReference<HiveTarget?>(null)
    private var paint: Paint? = null

    /** Call each loop with the latest [HiveScoring.update] result. */
    fun publish(target: HiveTarget?) {
        targetRef.set(target)
    }

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {}

    override fun processFrame(frame: Mat?, captureTimeNanos: Long): Any? = targetRef.get()

    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?,
    ) {
        val c = canvas ?: return
        val t = (userContext as? HiveTarget) ?: return

        val p = paint ?: Paint().apply {
            textSize = 26f
            isAntiAlias = true
            style = Paint.Style.FILL
            setShadowLayer(3f, 1f, 1f, Color.BLACK)
        }.also { paint = it }

        val zone = when {
            t.distanceCm < HiveScoringConfig.SHOOT_MIN_RANGE_CM -> ScoringZone.TOO_CLOSE
            t.distanceCm > ShooterModel.effectiveMaxRangeCm() -> ScoringZone.TOO_FAR
            abs(t.bearingDeg) > HiveScoringConfig.SHOOT_MAX_BEARING_DEG -> ScoringZone.IN_RANGE
            else -> ScoringZone.ALIGNED
        }
        p.color = when (zone) {
            ScoringZone.ALIGNED -> Color.GREEN
            ScoringZone.IN_RANGE -> Color.YELLOW
            ScoringZone.TOO_CLOSE, ScoringZone.TOO_FAR -> Color.RED
            else -> Color.WHITE
        }

        val text = String.format(
            Locale.US, "%s: %.0fcm %+.1f° %s",
            t.label, t.distanceCm, t.bearingDeg, zone.name,
        )
        val x = (t.screenX * scaleBmpPxToCanvasPx).toFloat() - 30f
        val y = (t.screenY * scaleBmpPxToCanvasPx).toFloat() - 40f
        c.drawText(text, x.coerceAtLeast(4f), y.coerceAtLeast(30f), p)
    }
}
