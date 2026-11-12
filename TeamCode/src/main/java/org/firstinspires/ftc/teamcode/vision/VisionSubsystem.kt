package org.firstinspires.ftc.teamcode.vision

import android.util.Size
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.CommandBuilder
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.nextftc.robot.Mechanism
import java.util.Locale
import java.util.concurrent.atomic.AtomicReference
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.FRONT_CAM_NAME
import org.firstinspires.ftc.teamcode.UP_CAM_NAME
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.PoseCorrector
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.VisionProcessor
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin

// ============================================================================
// SECTION 1: HybridPoseEstimator - two-camera fusion + Road Runner PoseCorrector
// ============================================================================



/**
 * Fuses the two cameras' visual odometry with Road Runner localization and
 * produces clamped, confidence-gated SE(2) pose corrections via [PoseCorrector].
 *
 * NOT absolute localization: the visual pose is dead-reckoned visual odometry
 * seeded at the Road Runner start pose; it corrects drift within a run only.
 *
 * NOT stereo/triangulation: the two cameras look at different parts of the
 * environment (ceiling vs. field structures) and never observe the same
 * feature, so no disparity/range triangulation is possible. Fusion happens at
 * the *motion* level: each camera yields an independent relative-motion
 * estimate, and translation/heading are weighted per camera by
 * [org.firstinspires.ftc.teamcode.vision.CameraExtrinsics.translationTrust] /
 * headingTrust — heading is depth-free (usable from any mount angle), while
 * translation trust depends on the constant-depth assumption holding.
 */
class HybridPoseEstimator(
    private val upProcessor: VisionCameraProcessor,
    private val frontProcessor: VisionCameraProcessor,
) : PoseCorrector {

    private val snapshotRef = AtomicReference<VisionSnapshot?>(null)

    // ---- State machine ----
    private var state = VisionState.SEARCHING
    private var stateEnteredNanos = 0L
    private var lastSwitchNanos = 0L
    private var lastGoodVisionNanos = 0L

    // ---- Visual pose dead reckoning ----
    private var visualPose = Pose2d(0.0, 0.0, 0.0)
    private var upLastConsumedNanos = 0L
    private var frontLastConsumedNanos = 0L
    private var visionLostSinceNanos = 0L
    private var overallConfidence = 0.0

    private var fusedMotion: RobotMotionDelta? = null

    // ===== PUSH-RESISTANT FUSION =====
    /** True when the robot is being pushed (external force detected). */
    var isBeingPushed: Boolean = false

    /** Seeds the visual pose at the Road Runner pose (start of autonomous). */
    fun seed(rrPose: Pose2d) {
        visualPose = rrPose
        upLastConsumedNanos = 0L
        frontLastConsumedNanos = 0L
        visionLostSinceNanos = 0L
    }

    /**
     * Advances the fusion. Call once per control-loop iteration (from
     * [VisionSubsystem.periodic]). [rrPose] is only used to re-anchor the
     * visual estimate after sustained vision loss.
     */
    fun update(nowNanos: Long, rrPose: Pose2d) {
        val up = fresh(upProcessor.latestObservation(), nowNanos)
        val front = fresh(frontProcessor.latestObservation(), nowNanos)

        val upConf = up?.confidence ?: 0.0
        val frontConf = front?.confidence ?: 0.0
        val strong = max(upConf, frontConf)
        val weak = min(upConf, frontConf)
        overallConfidence = (strong + 0.5 * weak) / 1.5

        // ---- 1. Fuse fresh camera motions, per channel ----
        // Translation and heading are weighted SEPARATELY, because their
        // reliability depends on different geometry:
        //   - HEADING (image rotation) is depth-free and exact under yaw, so
        //     any camera with a clear stationary view contributes it fully
        //     (CameraExtrinsics.headingTrust, typically 1.0).
        //   - TRANSLATION (pixels -> cm) relies on the constant-depth
        //     assumption, which only holds for a camera roughly perpendicular
        //     to its scene plane (CameraExtrinsics.translationTrust; an
        //     angled-up camera sees features at many depths, so it defaults
        //     to 0 and contributes heading only).
        var wTrans = 0.0
        var wHead = 0.0
        var dx = 0.0
        var dy = 0.0
        var dh = 0.0
        val active = mutableSetOf<CameraId>()
        if (up != null && up.confidence >= VisionTuning.FUSION_MIN_CONFIDENCE) {
            val m = up.motion
            if (m != null && m.valid && m.timestampNanos != upLastConsumedNanos) {
                val cal = VisionConfig.calibrationFor(CameraId.UP).extrinsics
                val wt = up.confidence * cal.translationTrust
                val wh = up.confidence * cal.headingTrust
                wTrans += wt
                dx += wt * m.dxIn
                dy += wt * m.dyIn
                wHead += wh
                dh += wh * m.dHeadingRad
                active += up.cameraId
                upLastConsumedNanos = m.timestampNanos
            }
        }
        if (front != null && front.confidence >= VisionTuning.FUSION_MIN_CONFIDENCE) {
            val m = front.motion
            if (m != null && m.valid && m.timestampNanos != frontLastConsumedNanos) {
                val cal = VisionConfig.calibrationFor(CameraId.FRONT).extrinsics
                val wt = front.confidence * cal.translationTrust
                val wh = front.confidence * cal.headingTrust
                wTrans += wt
                dx += wt * m.dxIn
                dy += wt * m.dyIn
                wHead += wh
                dh += wh * m.dHeadingRad
                active += front.cameraId
                frontLastConsumedNanos = m.timestampNanos
            }
        }

        // A camera contributed if either channel has weight this update.
        val anyContribution = wTrans > 0.0 || wHead > 0.0

        fusedMotion = if (anyContribution) {
            RobotMotionDelta(
                dxIn = if (wTrans > 0.0) dx / wTrans else 0.0,
                dyIn = if (wTrans > 0.0) dy / wTrans else 0.0,
                dHeadingRad = if (wHead > 0.0) dh / wHead else 0.0,
                confidence = overallConfidence,
                timestampNanos = nowNanos,
                valid = true,
            )
        } else {
            null
        }

        // ---- 2. Integrate into the dead-reckoned visual pose ----
        if (anyContribution) {
            val d = fusedMotion!!
            visualPose = visualPose.plus(Twist2d(Vector2d(d.dxIn, d.dyIn), d.dHeadingRad))
            lastGoodVisionNanos = nowNanos
        }

        // ---- 3. Re-anchor after sustained vision loss (never apply stale jumps) ----
        if (anyContribution) {
            visionLostSinceNanos = 0L
        } else if (visionLostSinceNanos == 0L) {
            visionLostSinceNanos = nowNanos
        } else if (msBetween(visionLostSinceNanos, nowNanos) >= VisionTuning.FUSION_RESEED_MS) {
            visualPose = rrPose
        }

        // ---- 4. State machine with hysteresis ----
        updateState(nowNanos)

        // ---- 5. Publish the combined immutable snapshot ----
        snapshotRef.set(
            VisionSnapshot(
                timestampNanos = nowNanos,
                up = up,
                front = front,
                state = state,
                overallConfidence = overallConfidence,
                fusedMotion = fusedMotion,
                visualPoseXIn = visualPose.position.x,
                visualPoseYIn = visualPose.position.y,
                visualPoseHeadingRad = visualPose.heading.toDouble(),
                activeCameras = active,
            ),
        )
    }

    // ---- Hysteresis state machine ------------------------------------

    private fun updateState(nowNanos: Long) {
        if (stateEnteredNanos == 0L) {
            stateEnteredNanos = nowNanos
            lastSwitchNanos = nowNanos
        }
        val dwellOk = msBetween(lastSwitchNanos, nowNanos) >= VisionTuning.STATE_DWELL_MS

        val usableVision = overallConfidence >= VisionTuning.FUSION_MIN_CONFIDENCE
        val lostForMs = if (usableVision) 0.0 else msBetween(lastGoodVisionNanos, nowNanos)

        if (!dwellOk) return

        val nextState = when (state) {
            VisionState.SEARCHING -> when {
                overallConfidence >= VisionTuning.STATE_ACQUIRE_CONFIDENCE -> VisionState.ACQUIRING
                else -> state
            }
            VisionState.ACQUIRING -> when {
                overallConfidence >= VisionTuning.STATE_TRACK_CONFIDENCE -> VisionState.TRACKING
                overallConfidence < VisionTuning.STATE_ACQUIRE_CONFIDENCE - VisionTuning.STATE_DROP_MARGIN -> VisionState.SEARCHING
                else -> state
            }
            VisionState.TRACKING -> when {
                overallConfidence >= VisionTuning.STATE_HIGH_CONFIDENCE -> VisionState.HIGH_CONFIDENCE
                overallConfidence < VisionTuning.STATE_TRACK_CONFIDENCE - VisionTuning.STATE_DROP_MARGIN -> VisionState.ACQUIRING
                else -> state
            }
            VisionState.HIGH_CONFIDENCE -> when {
                overallConfidence < VisionTuning.STATE_HIGH_CONFIDENCE - VisionTuning.STATE_DROP_MARGIN -> VisionState.TRACKING
                else -> state
            }
            VisionState.LOST -> when {
                usableVision && msBetween(lastGoodVisionNanos, nowNanos) >= VisionTuning.STATE_RECOVER_MS -> VisionState.RECOVERING
                else -> state
            }
            VisionState.RECOVERING -> when {
                overallConfidence >= VisionTuning.STATE_TRACK_CONFIDENCE -> VisionState.TRACKING
                lostForMs >= VisionTuning.STATE_LOST_AFTER_MS -> VisionState.LOST
                else -> state
            }
        }

        // LOST overrides everything when vision has been unusable for a while
        val finalState =
            if (state != VisionState.LOST && lostForMs >= VisionTuning.STATE_LOST_AFTER_MS) {
                VisionState.LOST
            } else {
                nextState
            }

        if (finalState != state) {
            state = finalState
            stateEnteredNanos = nowNanos
            lastSwitchNanos = nowNanos
        }
    }

    // ---- PoseCorrector: the Road Runner integration point ----------------

    /**
     * Computes the confidence-gated, clamped SE(2) correction. Returns
     * [rrPose] UNTOUCHED whenever:
     *  - the state is not TRUSTING (TRACKING / HIGH_CONFIDENCE / RECOVERING),
     *  - the overall confidence is below the fusion floor,
     *  - the visual-vs-RoadRunner error exceeds the jump limits (the visual
     *    estimate is re-seeded instead of trusted),
     *  - the state machine has been in a trusting state for less than the
     *    dwell time (single good frames can never move the pose).
     */
    override fun correct(rrPose: Pose2d, rrVelocity: PoseVelocity2d?, timestampNanos: Long): Pose2d {
        val trusting = state == VisionState.TRACKING ||
            state == VisionState.HIGH_CONFIDENCE ||
            state == VisionState.RECOVERING

        if (!trusting || overallConfidence < VisionTuning.FUSION_MIN_CONFIDENCE) {
            return rrPose
        }

        // Require the trusting state to have persisted (single good frames can
        // never move the pose).
        if (stateEnteredNanos == 0L ||
            msBetween(stateEnteredNanos, timestampNanos) < VisionTuning.STATE_DWELL_MS
        ) {
            return rrPose
        }

        val err = visualPose.minus(rrPose) // Twist2d taking rrPose to visualPose

        if (abs(err.angle) > Math.toRadians(VisionTuning.FUSION_MAX_ERR_DEG) ||
            hypot(err.line.x, err.line.y) > VisionTuning.FUSION_MAX_ERR_IN
        ) {
            // Visual estimate diverged — re-anchor instead of applying a jump.
            visualPose = rrPose
            return rrPose
        }

        // Confidence-scaled weight, capped so vision can never dominate RR.
        // When pushed, trust vision corrections more (higher gain, larger steps).
        val maxGain: Double
        val maxStepIn: Double
        val maxStepDeg: Double
        if (isBeingPushed) {
            maxGain = org.firstinspires.ftc.teamcode.PUSH_FUSION_MAX_GAIN
            maxStepIn = org.firstinspires.ftc.teamcode.PUSH_FUSION_MAX_STEP_IN
            maxStepDeg = org.firstinspires.ftc.teamcode.PUSH_FUSION_MAX_STEP_DEG
        } else {
            maxGain = VisionTuning.FUSION_MAX_GAIN
            maxStepIn = VisionTuning.FUSION_MAX_STEP_IN
            maxStepDeg = VisionTuning.FUSION_MAX_STEP_DEG
        }

        val weight = min(maxGain, overallConfidence * maxGain)

        val cappedLine = Vector2d(
            (weight * err.line.x).coerceIn(-maxStepIn, maxStepIn),
            (weight * err.line.y).coerceIn(-maxStepIn, maxStepIn),
        )
        val cappedAngle = (weight * err.angle).coerceIn(
            -Math.toRadians(maxStepDeg),
            Math.toRadians(maxStepDeg),
        )

        return rrPose.plus(Twist2d(cappedLine, cappedAngle))
    }

    // ---- Helpers / accessors ------------------------------------------

    /** Rejects snapshots older than CONF_STALE_AFTER_MS. */
    private fun fresh(obs: CameraObservation?, nowNanos: Long): CameraObservation? {
        if (obs == null) return null
        val ageMs = msBetween(obs.timestampNanos, nowNanos)
        return if (ageMs <= VisionTuning.CONF_STALE_AFTER_MS && ageMs >= 0) obs else null
    }

    private fun msBetween(a: Long, b: Long): Double = (b - a) / 1e6

    /** Latest combined snapshot for telemetry / the guidance command. */
    fun latestSnapshot(): VisionSnapshot? = snapshotRef.get()

    val currentState: VisionState get() = state
    val confidence: Double get() = overallConfidence
}


// ============================================================================
// SECTION 2: VisionSubsystem - NextFTC Mechanism owning both cameras
// ============================================================================



/**
 * NextFTC [Mechanism] that owns the two localization cameras and the hybrid
 * estimator. Fits the project's existing NextFTC architecture: it is a
 * subsystem with a [periodic] update, exposes reusable NextFTC commands
 * ([visionGuidanceCommand]), and never touches motors itself.
 *
 * Threading: image processing happens on the VisionPortal vision threads;
 * [periodic] only consumes the latest immutable snapshots, so the NextFTC
 * loop stays fast.
 */
/**
 * NextFTC [Mechanism] that owns the two localization cameras and the hybrid
 * estimator, and can additionally host known-target processors on the same
 * portals — the autonomous/teleop ecosystem puts the hive AprilTagProcessor
 * on the UP camera and the ball blob locators on the FRONT camera so a single
 * portal pair serves localization AND game-piece detection.
 */
class VisionSubsystem(
    hardwareMap: HardwareMap,
    extraUpProcessors: List<VisionProcessor> = emptyList(),
    extraFrontProcessors: List<VisionProcessor> = emptyList(),
) : Mechanism {

    val upProcessor = VisionCameraProcessor(CameraId.UP)
    val frontProcessor = VisionCameraProcessor(CameraId.FRONT)

    /** The two cameras are two observation sources for ONE pose estimate. */
    val fusion = HybridPoseEstimator(upProcessor, frontProcessor)

    /** Updates the push state on the fusion estimator for push-resistant corrections. */
    fun setPushState(pushed: Boolean) {
        fusion.isBeingPushed = pushed
    }

    // ---- SINGLE-CAMERA SUPPORT ----
    // One camera works: mount it in the FRONT position (as high as practical,
    // aimed ~level) and wire it under FRONT_CAM_NAME. It then carries the
    // front processors AND the UP camera's jobs (AprilTags etc.); localization
    // runs on one odometry source — fine, because the dead wheels + IMU now
    // carry the pose. A camera wired under UP_CAM_NAME only also works.

    /** True when the UP ("high") camera exists in the hardware configuration. */
    val hasUpCamera: Boolean = cameraExists(hardwareMap, UP_CAM_NAME)

    /** True when the FRONT ("ground") camera exists in the hardware configuration. */
    val hasFrontCamera: Boolean = cameraExists(hardwareMap, FRONT_CAM_NAME)

    /**
     * Which camera the hive AprilTagProcessor is actually attached to (the
     * intrinsics HiveScoring must use for range/bearing). Tags ride the UP
     * portal when it exists; in single-camera mode they follow the survivors.
     */
    val aprilTagCameraId: CameraId = if (hasUpCamera) CameraId.UP else CameraId.FRONT

    private val portalViewIds =
        VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL)

    val upPortal: VisionPortal? =
        if (hasUpCamera) {
            VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName::class.java, UP_CAM_NAME))
                .setCameraResolution(Size(VisionTuning.UP_CAM_WIDTH, VisionTuning.UP_CAM_HEIGHT))
                .setLiveViewContainerId(portalViewIds[0])
                .addProcessors(upProcessor, *extraUpProcessors.toTypedArray())
                .build()
        } else {
            null
        }

    val frontPortal: VisionPortal? =
        if (hasFrontCamera) {
            VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName::class.java, FRONT_CAM_NAME))
                .setCameraResolution(Size(VisionTuning.FRONT_CAM_WIDTH, VisionTuning.FRONT_CAM_HEIGHT))
                .setLiveViewContainerId(portalViewIds[1])
                .addProcessors(
                    frontProcessor,
                    *extraFrontProcessors.toTypedArray(),
                    // Single-camera mode: the surviving front camera also
                    // carries the UP camera's processors (AprilTags, overlays).
                    *(if (hasUpCamera) emptyList() else extraUpProcessors).toTypedArray(),
                )
                .build()
        } else {
            null
        }

    /** Compact camera-mode line for telemetry. */
    val cameraMode: String = when {
        hasUpCamera && hasFrontCamera -> "2 cameras (up+front hybrid)"
        hasUpCamera -> "1 camera [UP] — front jobs ride on it"
        hasFrontCamera -> "1 camera [FRONT] — up jobs ride on it"
        else -> "NO CAMERAS — vision inactive"
    }

    private var lastRrPose = Pose2d(0.0, 0.0, 0.0)

    /** True once a real Road Runner pose has been provided (teleop has none). */
    private var poseProvided = false

    /**
     * Wires the fusion into the Road Runner drive at the supported hook.
     * Corrections stay disabled until tests pass (FUSION_ENABLED_DEFAULT);
     * enable per-OpMode with [MecanumDrive.visionCorrectionEnabled].
     */
    fun attachTo(drive: MecanumDrive, enableCorrections: Boolean = VisionTuning.FUSION_ENABLED_DEFAULT) {
        drive.poseCorrector = fusion
        drive.visionCorrectionEnabled = enableCorrections
        fusion.seed(drive.pose)
    }

    /**
     * NextFTC mechanism periodic: advances fusion with the latest Road Runner
     * pose. Called automatically when the NextFTC scheduler loop runs the
     * robot's mechanisms, or manually from LinearOpMode loops. Without a real
     * pose (pure teleop), the world landmark map stays idle instead of
     * building a map against a fake origin.
     */
    override fun periodic() {
        if (poseProvided) {
            upProcessor.updateRobotPose(lastRrPose)
            frontProcessor.updateRobotPose(lastRrPose)
        }
        fusion.update(System.nanoTime(), lastRrPose)
    }

    /** Feed the current Road Runner pose so the estimator can re-anchor. */
    fun updateRoadRunnerPose(pose: Pose2d) {
        lastRrPose = pose
        poseProvided = true
        upProcessor.updateRobotPose(pose)
        frontProcessor.updateRobotPose(pose)
        fusion.update(System.nanoTime(), pose)
    }

    fun snapshot(): VisionSnapshot? = fusion.latestSnapshot()

    /** Reusable NextFTC-compatible final-alignment command (see VisionGuidanceCommand.kt). */
    fun visionGuidanceCommand(
        drive: MecanumDrive,
        forwardOffsetIn: Double = 0.0,
        strafeOffsetIn: Double = 0.0,
        headingOffsetDeg: Double = 0.0,
    ) = visionGuidance(drive, this, forwardOffsetIn, strafeOffsetIn, headingOffsetDeg)

    /** Full telemetry dashboard (see build request section 25). */
    fun addTelemetry(t: Telemetry) {
        val s = fusion.latestSnapshot()
        val fmt = { v: Double -> String.format(Locale.US, "%.2f", v) }

        for (obs in listOfNotNull(s?.up, s?.front)) {
            val label = if (obs.cameraId == CameraId.UP) "UP CAM" else "FRONT CAM"
            t.addData("$label fps", fmt(obs.fps))
            t.addData("$label detected", obs.detectedCount)
            t.addData("$label tracked", obs.trackedCount)
            t.addData("$label RANSAC inliers", obs.inlierCount)
            t.addData("$label RANSAC outliers", obs.outlierCount)
            t.addData("$label confidence", fmt(obs.confidence))
            t.addData(
                "$label landmarks (world-mapped)",
                "${obs.landmarks.size} (${obs.landmarks.count { it.worldXIn != null }})",
            )
            // A few sample world-mapped landmark coordinates, for map reference.
            obs.landmarks.filter { it.worldXIn != null }.take(3).forEach { lm ->
                t.addData(
                    "  lm#${lm.id} world (in)",
                    "%.1f, %.1f, %.1f".format(lm.worldXIn, lm.worldYIn, lm.worldZIn),
                )
            }
        }
        if (s == null) {
            t.addData("VISION", "no snapshot yet")
            return
        }
        t.addData("VISION state", s.state)
        t.addData("VISION confidence", fmt(s.overallConfidence))
        t.addData("VISION active cams", s.activeCameras.joinToString(","))
        val m = s.fusedMotion
        if (m != null && m.valid) {
            t.addData("VISUAL dx", fmt(m.dxIn))
            t.addData("VISUAL dy", fmt(m.dyIn))
            t.addData("VISUAL dHeading", String.format(Locale.US, "%.1f deg", Math.toDegrees(m.dHeadingRad)))
        }
        t.addData("VISUAL pose", "${fmt(s.visualPoseXIn)}, ${fmt(s.visualPoseYIn)}, " +
            String.format(Locale.US, "%.1f deg", Math.toDegrees(s.visualPoseHeadingRad)))
    }

    fun close() {
        upPortal?.close()
        frontPortal?.close()
    }

    private fun cameraExists(hardwareMap: HardwareMap, name: String): Boolean =
        try {
            hardwareMap.get(WebcamName::class.java, name)
            true
        } catch (_: Exception) {
            false
        }
}

// ============================================================================
// SECTION 3: visionGuidance - reusable NextFTC final-alignment command
// ============================================================================



/**
 * Reusable NextFTC/Ivy-compatible final-alignment command.
 *
 * PURPOSE: after a Road Runner trajectory, re-servo the robot to the (vision-
 * corrected) fused pose so the final alignment is anchored against the
 * stationary visual references the cameras have been tracking. Because the
 * generic vision system provides *relative* motion (visual odometry), the
 * alignment target is captured when the command starts — the command removes
 * residual drift/overshoot relative to the best available pose estimate,
 * which is exactly what vision correction improves.
 *
 * Safety properties:
 *  - refuses to move when vision confidence is below GUIDANCE_CONFIDENCE_FLOOR
 *    (holds zero power; the timeout still applies),
 *  - never trusts a single good frame: it only terminates after
 *    [VisionTuning.GUIDANCE_SETTLE_FRAMES] consecutive in-tolerance frames,
 *  - terminates on timeout with motors zeroed,
 *  - all axes (forward/strafe/turn) have independent gains, tolerances and
 *    power caps, and commands are rate-limited.
 *
 * Usage (NextFTC style):
 *   val cmd = visionGuidance(drive, vision)
 *   cmd.schedule()                 // with the Scheduler running, or inline:
 *   cmd.start(); while (!cmd.done()) { Scheduler.execute() }; cmd.end(NATURALLY)
 */
fun visionGuidance(
    drive: MecanumDrive,
    vision: VisionSubsystem,
    forwardOffsetIn: Double = 0.0,
    strafeOffsetIn: Double = 0.0,
    headingOffsetDeg: Double = 0.0,
): Command {
    var target: Pose2d? = null
    var beginNanos = 0L
    var settleFrames = 0
    var lastForward = 0.0
    var lastStrafe = 0.0
    var lastTurn = 0.0
    var confidenceOk = false

    val builder: CommandBuilder = Command.build()
        .requiring(drive, vision)

    builder.setStart {
        beginNanos = System.nanoTime()
        settleFrames = 0
        lastForward = 0.0; lastStrafe = 0.0; lastTurn = 0.0
        // Target is the current (possibly vision-corrected) pose plus offsets.
        val p = drive.pose
        val off = Math.toRadians(headingOffsetDeg)
        target = Pose2d(
            Vector2d(
                p.position.x + forwardOffsetIn * cos(p.heading.toDouble()) - strafeOffsetIn * sin(p.heading.toDouble()),
                p.position.y + forwardOffsetIn * sin(p.heading.toDouble()) + strafeOffsetIn * cos(p.heading.toDouble()),
            ),
            p.heading.toDouble() + off,
        )
    }

    builder.setExecute {
        vision.updateRoadRunnerPose(drive.pose) // keep fusion fed inside the command
        val snap = vision.snapshot()

        val conf = snap?.overallConfidence ?: 0.0
        val stale = snap == null ||
            (System.nanoTime() - snap.timestampNanos) / 1e6 > VisionTuning.CONF_STALE_AFTER_MS
        confidenceOk = conf >= VisionTuning.GUIDANCE_CONFIDENCE_FLOOR && !stale

        val tgt = target ?: return@setExecute

        if (!confidenceOk) {
            // Vision unusable: hold still, do not count settle frames.
            drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
            settleFrames = 0
            return@setExecute
        }

        val err = tgt.minus(drive.pose) // body-frame twist: +x fwd, +y left, +angle CCW

        fun limit(old: Double, new: Double): Double {
            val rate = VisionTuning.GUIDANCE_RATE_LIMIT
            return old + (new - old).coerceIn(-rate, rate)
        }

        val forward = limit(
            lastForward,
            (VisionTuning.GUIDANCE_K_FORWARD * err.line.x)
                .coerceIn(-VisionTuning.GUIDANCE_MAX_FORWARD_POWER, VisionTuning.GUIDANCE_MAX_FORWARD_POWER),
        )
        val strafe = limit(
            lastStrafe,
            (VisionTuning.GUIDANCE_K_STRAFE * err.line.y)
                .coerceIn(-VisionTuning.GUIDANCE_MAX_STRAFE_POWER, VisionTuning.GUIDANCE_MAX_STRAFE_POWER),
        )
        val turn = limit(
            lastTurn,
            (VisionTuning.GUIDANCE_K_TURN * Math.toDegrees(err.angle))
                .coerceIn(-VisionTuning.GUIDANCE_MAX_TURN_POWER, VisionTuning.GUIDANCE_MAX_TURN_POWER),
        )
        lastForward = forward; lastStrafe = strafe; lastTurn = turn

        drive.setDrivePowers(PoseVelocity2d(Vector2d(forward, strafe), turn))

        val posErr = kotlin.math.hypot(err.line.x, err.line.y)
        val headingErrDeg = abs(Math.toDegrees(err.angle))
        val inTolerance = posErr <= VisionTuning.GUIDANCE_POS_TOLERANCE_IN &&
            headingErrDeg <= VisionTuning.GUIDANCE_HEADING_TOLERANCE_DEG
        settleFrames = if (inTolerance) settleFrames + 1 else 0
    }

    builder.setDone {
        if (settleFrames >= VisionTuning.GUIDANCE_SETTLE_FRAMES) return@setDone true
        // Timeout with motors safely zeroed.
        (System.nanoTime() - beginNanos) / 1e6 >= VisionTuning.GUIDANCE_TIMEOUT_MS
    }

    builder.setEnd {
        drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
    }

    return builder
}
