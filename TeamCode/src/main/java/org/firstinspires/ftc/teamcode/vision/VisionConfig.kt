package org.firstinspires.ftc.teamcode.vision

import com.acmerobotics.dashboard.config.Config

// ============================================================================
// SECTION 1: Immutable snapshot types shared across the vision pipeline
// ============================================================================


/**
 * Immutable, thread-safe data types shared across the vision pipeline.
 * Everything the control loop touches is one of these snapshots; mutable
 * tracking state stays inside the vision thread (processors / trackers).
 */

/** Identifies the two physical cameras with intentionally different views. */
enum class CameraId { UP, FRONT }

/** Vision state machine states. Transitions use hysteresis (see HybridPoseEstimator). */
enum class VisionState {
    SEARCHING,       // no features / no RANSAC consensus yet
    ACQUIRING,       // low-confidence tracking, building up observations
    TRACKING,        // consistent static-scene tracking
    HIGH_CONFIDENCE, // many agreeing, persistent features; corrections allowed
    LOST,            // stale snapshots / RANSAC failing / cameras blocked
    RECOVERING,      // vision came back after LOST, rebuilding trust
}

/** Simple immutable 2D pixel coordinate (OpenCV Point is mutable, so we don't share it). */
data class PixelPoint(val x: Double, val y: Double)

/** Immutable snapshot of a single tracked visual feature. */
data class TrackedFeature(
    val id: Long,
    val cameraId: CameraId,
    val previousPixel: PixelPoint,
    val currentPixel: PixelPoint,
    val ageFrames: Int,
    val successfulFrames: Int,
    val trackingErrorPx: Double,
    /** EMA of how often this feature agreed with the dominant scene motion (0..1). */
    val staticConsistency: Double,
    /** True when the most recent RANSAC classified it as part of the static scene. */
    val isStaticInlier: Boolean,
    val isNew: Boolean,
    val lastSeenTimestampNanos: Long,
)

/**
 * Relative robot motion measured by vision, expressed in the ROBOT frame:
 * +x forward, +y left, positive heading = CCW (Road Runner convention).
 * Units: inches / radians. This is *relative* motion (visual odometry), NOT an
 * absolute field position.
 */
data class RobotMotionDelta(
    val dxIn: Double,
    val dyIn: Double,
    val dHeadingRad: Double,
    val confidence: Double,
    val timestampNanos: Long,
    val valid: Boolean,
)

/**
 * Immutable per-camera result snapshot produced by the vision thread and
 * consumed by the control loop. A stale [timestampNanos] means the camera is
 * blocked or its portal died.
 */
data class CameraObservation(
    val cameraId: CameraId,
    val timestampNanos: Long,
    val fps: Double,
    val detectedCount: Int,
    val trackedCount: Int,
    val inlierCount: Int,
    val outlierCount: Int,
    val ransacOk: Boolean,
    val inlierRatio: Double,
    val meanTrackErrorPx: Double,
    /** 0..1 per-camera confidence (EMA-smoothed). */
    val confidence: Double,
    /** Robot-frame motion from this camera, or null if the frame estimate failed. */
    val motion: RobotMotionDelta?,
    val features: List<TrackedFeature>,
    val landmarks: List<VisualLandmark>,
)

/**
 * A persistent *visual* landmark: a cluster of long-lived static features.
 * No semantic identity is claimed — "landmark 17" simply means "persistent
 * static visual structure", not "rafter" or "wall".
 */
data class VisualLandmark(
    val id: Long,
    val cameraId: CameraId,
    val centerPixel: PixelPoint,
    val featureCount: Int,
    val ageFrames: Int,
    val observations: Int,
    /** 0..1 stability score (persistence * motion consistency). */
    val stabilityScore: Double,
    val confidence: Double,
    /**
     * World-frame position (inches, anchored to the robot's start pose),
     * triangulated from bearing rays observed as the robot moved — null until
     * the landmark has accumulated enough parallax. This is the "self-built
     * map for reference": no semantics, just where persistent static structure
     * sits relative to where the run began.
     */
    val worldXIn: Double? = null,
    val worldYIn: Double? = null,
    val worldZIn: Double? = null,
)

/** Immutable combined snapshot the control/telemetry side consumes. */
data class VisionSnapshot(
    val timestampNanos: Long,
    val up: CameraObservation?,
    val front: CameraObservation?,
    val state: VisionState,
    /** 0..1 combined confidence across active cameras. */
    val overallConfidence: Double,
    /** Fused robot-frame visual motion for the latest update (null if none valid). */
    val fusedMotion: RobotMotionDelta?,
    /** Dead-reckoned visual pose estimate (world frame, starts at RR start pose). */
    val visualPoseXIn: Double,
    val visualPoseYIn: Double,
    val visualPoseHeadingRad: Double,
    /** Which cameras contributed to the current visual estimate. */
    val activeCameras: Set<CameraId>,
)

// ============================================================================
// SECTION 2: All vision algorithm tuning constants
// ============================================================================



/**
 * ============ VISION TUNING (algorithm parameters) ============
 *
 * All vision algorithm tuning lives here. Physical measurements (camera
 * intrinsics/extrinsics) live in [VisionConfig]. Values marked TUNE are safe
 * to adjust; they are exposed to FTC Dashboard where possible.
 *
 * Units: Road Runner uses INCHES/RADIANS; vision config uses pixels,
 * centimeters and milliseconds for the camera-facing layers, and converts at
 * the [VisualMotionEstimator] boundary.
 */
@Config
object VisionTuning {
    // ---------- Feature detection (Shi-Tomasi) ----------
    const val MAX_FEATURES = 60              // per camera, per detection
    const val DETECTION_QUALITY = 0.05       // goodFeaturesToTrack qualityLevel
    const val DETECTION_MIN_DISTANCE_PX = 15.0
    const val DETECTION_BLOCK_SIZE = 10

    // ---------- Optical flow (pyramidal Lucas-Kanade) ----------
    const val LK_WINDOW_SIZE = 21             // window (odd)
    const val LK_PYRAMID_LEVELS = 3
    const val LK_MAX_ERROR_PX = 4.0           // per-frame LK error gate
    const val LK_MIN_TRACKS = 25              // below this, reacquire new features
    const val LK_REACQUIRE_INTERVAL_FRAMES = 15
    const val LK_MAX_FEATURE_AGE_FRAMES = 90  // force-refresh long-lived tracks
    const val LK_MIN_SUCCESS_FRAMES = 5       // before a track can be "mature"

    // ---------- RANSAC static-scene estimation ----------
    const val RANSAC_ITERATIONS = 40
    const val RANSAC_INLIER_THRESHOLD_PX = 3.0
    const val RANSAC_MIN_INLIERS = 12
    const val RANSAC_MIN_INLIER_RATIO = 0.45
    const val RANSAC_MAX_SCALE_ERROR = 0.10   // |scale-1| beyond this rejects the frame

    // ---------- Landmark persistence ----------
    const val LANDMARK_CLUSTER_RADIUS_PX = 40.0
    const val LANDMARK_MAX = 20               // per camera
    const val LANDMARK_MIN_OBSERVATIONS = 8
    const val LANDMARK_MIN_AGE_FRAMES = 15

    // ---------- Per-camera confidence ----------
    const val CONF_EMA_ALPHA = 0.15          // smoothing (lower = smoother)
    const val CONF_MIN_TRACKS_FOR_FULL = 40.0
    const val CONF_STALE_AFTER_MS = 500.0    // snapshot age beyond which camera counts as lost

    // ---------- State machine hysteresis ----------
    const val STATE_ACQUIRE_CONFIDENCE = 0.25 // SEARCHING -> ACQUIRING
    const val STATE_TRACK_CONFIDENCE = 0.45   // ACQUIRING -> TRACKING
    const val STATE_HIGH_CONFIDENCE = 0.70    // TRACKING -> HIGH_CONFIDENCE
    const val STATE_DROP_MARGIN = 0.15        // exit threshold = enter - margin (hysteresis)
    const val STATE_DWELL_MS = 400.0          // min time in a state before switching again
    const val STATE_LOST_AFTER_MS = 1000.0    // no usable vision for this long -> LOST
    const val STATE_RECOVER_MS = 600.0         // LOST -> RECOVERING when vision returns

    // ---------- Pose fusion ----------
    const val FUSION_ENABLED_DEFAULT = false // master gate: must pass field tests first!
    const val FUSION_MIN_CONFIDENCE = 0.35    // below this, vision never moves the pose
    const val FUSION_MAX_GAIN = 0.20          // max fraction of error corrected per update
    const val FUSION_MAX_STEP_IN = 0.30       // max translational correction per update
    const val FUSION_MAX_STEP_DEG = 2.0       // max heading correction per update (deg)
    const val FUSION_MAX_ERR_IN = 12.0       // errors larger than this are rejected as jumps
    const val FUSION_MAX_ERR_DEG = 15.0      // heading equivalent
    const val FUSION_RESEED_MS = 2500.0       // confidence lost this long: re-seed visual pose

    // ---------- Vision guidance command ----------
    const val GUIDANCE_K_FORWARD = 0.10       // power per inch of error
    const val GUIDANCE_K_STRAFE = 0.10
    const val GUIDANCE_K_TURN = 0.02          // power per degree of heading error
    const val GUIDANCE_POS_TOLERANCE_IN = 0.75
    const val GUIDANCE_HEADING_TOLERANCE_DEG = 3.0
    const val GUIDANCE_SETTLE_FRAMES = 5      // consecutive in-tolerance frames to finish
    const val GUIDANCE_TIMEOUT_MS = 4000.0
    const val GUIDANCE_CONFIDENCE_FLOOR = 0.45 // refuse to move below this confidence
    const val GUIDANCE_MAX_FORWARD_POWER = 0.25
    const val GUIDANCE_MAX_STRAFE_POWER = 0.25
    const val GUIDANCE_MAX_TURN_POWER = 0.20
    const val GUIDANCE_RATE_LIMIT = 0.05       // max power change per loop (accel limiting)

    // ---------- Camera stream ----------
    // UP camera at 800x448: more pixels = more AprilTag range, and LK with ~60
    // features still fits the 30 fps budget. (C270 supports 640x480, 800x448
    // and 1280x720.) FRONT stays at 640x480 where the blob locators' focal
    // constant (C270_FOCAL_LENGTH_PIXELS) was calibrated.
    const val UP_CAM_WIDTH = 800
    const val UP_CAM_HEIGHT = 448
    const val FRONT_CAM_WIDTH = 640
    const val FRONT_CAM_HEIGHT = 480
    // Full-rate tracking (process EVERY frame): small inter-frame displacements
    // keep Lucas-Kanade locked on its features. Two 640x480 pipelines with ~60
    // features each fit the control hub budget at 30 fps; if the FPS telemetry
    // sags, raise this to 2 (~15 Hz) — tracking tolerates it, it just
    // reacquires more often.
    const val PROCESS_EVERY_NTH_FRAME = 1
    const val ADAPTIVE_FRAME_DECIMATION = true     // enable adaptive frame rate
    const val ADAPTIVE_DECIMATION_HIGH_CONF = 1   // process every frame when confidence high
    const val ADAPTIVE_DECIMATION_MED_CONF = 2    // process every 2nd frame when confidence medium
    const val ADAPTIVE_DECIMATION_LOW_CONF = 3    // process every 3rd frame when confidence low
    const val ADAPTIVE_HIGH_CONF_THRESHOLD = 0.7  // above this = high confidence
    const val ADAPTIVE_LOW_CONF_THRESHOLD = 0.35  // below this = low confidence
    const val DEBUG_OVERLAY = true            // draw tracked features / RANSAC classes

    // ---------- Self-building world map (start-pose anchored, reference only) ----------
    // Because the robot starts at a known fixed spot, persistent landmarks can
    // be triangulated into a WORLD frame via bearing-ray intersection as the
    // robot moves (parallax replaces the assumed depth). The map is used for
    // reference/telemetry and future reacquisition aids — it does not yet feed
    // back into pose correction.
    const val WORLD_MAP_ENABLED = true
    const val WORLD_MAP_MIN_BASELINE_IN = 3.0  // camera travel needed before triangulating
    const val WORLD_MAP_MAX_OBSERVATIONS = 12  // bounded least-squares memory per landmark
}

// ============================================================================
// SECTION 3: Physical camera calibration (everything MEASURE-marked)
// ============================================================================


/**
 * ============ CAMERA CALIBRATION (physical measurements) ============
 *
 * Every value in this file marked MEASURE must be measured on the physical
 * robot / venue. Nothing here should be hardcoded elsewhere.
 *
 * Coordinate conventions (documented, not guessed):
 *  - Pixel space: x right, y down (OpenCV).
 *  - Robot space: +x forward, +y left, heading CCW positive (Road Runner).
 *  - [imageToRobotYawDeg] is the direction (in robot frame, degrees CCW from
 *    forward) that "image rightward feature motion" maps to when the robot
 *    translates. Calibrate with Test 3 (see VisualLocalizationTest doc):
 *    push the robot straight forward; the reported motion must read +x.
 */

/** Pinhole camera intrinsics in pixels at the configured resolution. */
data class CameraIntrinsics(
    /** MEASURE: horizontal focal length (pixels). C270 @640x480 is ~543 px (see globals). */
    val focalLengthPx: Double,
    /** MEASURE: optical center x (pixels); approx width/2 if not calibrated. */
    val centerX: Double,
    /** MEASURE: optical center y (pixels); approx height/2 if not calibrated. */
    val centerY: Double,
    /** MEASURE: radial distortion k1,k2,k3 + tangential p1,p2; empty = ignore. */
    val distortion: List<Double> = emptyList(),
)

/**
 * Extrinsics: where the camera sits on the robot and which direction it looks.
 * The 3D pose fields (position / orientation) describe the mount for future 3D
 * pipelines; the current planar motion estimator additionally uses the
 * projection parameters below.
 */
data class CameraExtrinsics(
    /** MEASURE: camera position relative to robot center, cm. +x forward, +y left, +z up. */
    val positionXcm: Double,
    val positionYcm: Double,
    val positionZcm: Double,
    /** MEASURE: orientation of the camera optical axis, degrees; yaw CCW. */
    val yawDeg: Double,
    val pitchDeg: Double,
    val rollDeg: Double,
    /**
     * MEASURE (Test 3 calibration): angle, in robot frame, that corresponds to
     * image-rightward motion of the scene when the ROBOT translates.
     */
    val imageToRobotYawDeg: Double,
    /**
     * MEASURE + TUNE: assumed perpendicular distance from camera to the observed
     * dominant plane, in cm. This is the monocular depth assumption —
     * translation accuracy is only as good as this value.
     *
     * IMPORTANT: this is an *effective scale factor*, not necessarily a literal
     * physical distance. It is only literally true if the camera is mounted
     * roughly PERPENDICULAR to a planar scene (straight up at a flat ceiling,
     * or squarely at a wall). For an ANGLED camera the observed features sit
     * at many different depths, so no single number is "true" — calibrate it
     * empirically with Test 3: push the robot a known distance, compare with
     * the reported dx/dy, and adjust until they agree. If the scale is too
     * unstable to calibrate, set [translationTrust] to 0.0 instead and let the
     * camera contribute heading only.
     */
    val sceneDistanceCm: Double,
    /** +1 or -1: sign of the heading estimate (image rotation) mapped to robot yaw. */
    val headingSign: Double,
    /**
     * TUNE (0..1): how strongly this camera's TRANSLATION estimate is trusted
     * in fusion. Translation depends on the constant-depth assumption above,
     * so cameras viewing their scene at a shallow or angled direction (depth
     * varies across the image) should have LOW or ZERO translation trust.
     * 0.0 = this camera contributes heading only.
     */
    val translationTrust: Double,
    /**
     * TUNE (0..1): trust in this camera's HEADING (image rotation) estimate.
     * Heading is depth-free — the scene rotates exactly with robot yaw
     * regardless of feature distance — so this is typically 1.0 for any
     * camera with a clear stationary view.
     */
    val headingTrust: Double,
)

/** Full calibration of one camera. */
data class CameraCalibration(
    val cameraId: CameraId,
    val intrinsics: CameraIntrinsics,
    val extrinsics: CameraExtrinsics,
)

/**
 * Default calibration values. EVERY field is a MEASURE value — these defaults
 * are sane starting points (C270-class webcam, ~2.9 m ceiling) and must be
 * replaced with measured values for real accuracy.
 */
object VisionConfig {

    /**
     * UP camera — angled upward at the ceiling (rafters, trusses, lights).
     *
     * Realistically this camera will NOT be pointed straight up, so it views
     * features at varying depths. Therefore, by default it contributes
     * HEADING ONLY (image rotation is depth-free and exact under yaw); its
     * translation trust is 0.0. If your mounting happens to be nearly
     * perpendicular to a planar ceiling, calibrate sceneDistanceCm via Test 3
     * and raise translationTrust.
     */
    val UP_CAMERA = CameraCalibration(
        cameraId = CameraId.UP,
        intrinsics = CameraIntrinsics(
            // MEASURE: C270-class webcam at 800x448 (543 px focal @640 width scaled).
            focalLengthPx = 679.0,
            centerX = VisionTuning.UP_CAM_WIDTH / 2.0,
            centerY = VisionTuning.UP_CAM_HEIGHT / 2.0,
        ),
        extrinsics = CameraExtrinsics(
            positionXcm = 0.0,          // MEASURE
            positionYcm = 0.0,          // MEASURE
            positionZcm = 40.0,         // MEASURE: camera height above floor
            yawDeg = 0.0,               // MEASURE
            pitchDeg = 55.0,            // MEASURE: angled up at rafters (NOT straight up)
            rollDeg = 0.0,              // MEASURE
            // MEASURE (Test 3): ceiling rafters move opposite to robot motion.
            imageToRobotYawDeg = 90.0,  // starting guess; calibrate on the robot
            // Only meaningful while translationTrust > 0 (see CameraExtrinsics docs).
            sceneDistanceCm = 250.0,    // MEASURE + TUNE (Test 3): effective scale depth
            headingSign = -1.0,         // MEASURE (Test 2): scene rotates opposite to yaw
            translationTrust = 0.0,     // angled mount: depth varies across features
            headingTrust = 1.0,         // rotation is depth-free — full trust
        ),
    )

    /**
     * FRONT camera — low, facing forward at field walls/posts/structures.
     *
     * Carries the translation channel by default: walls are closer and more
     * fronto-parallel than the angled ceiling view, and the geometry of low
     * structures (posts, wall bases, corners) makes pixel motion a more stable
     * scale reference. Note wall distance still varies as the robot moves —
     * calibrate sceneDistanceCm at your typical working distance (Test 3) and
     * lower translationTrust if the venue has poor wall geometry.
     */
    val FRONT_CAMERA = CameraCalibration(
        cameraId = CameraId.FRONT,
        intrinsics = CameraIntrinsics(
            focalLengthPx = 543.0,      // MEASURE
            centerX = VisionTuning.FRONT_CAM_WIDTH / 2.0,
            centerY = VisionTuning.FRONT_CAM_HEIGHT / 2.0,
        ),
        extrinsics = CameraExtrinsics(
            positionXcm = 15.0,         // MEASURE
            positionYcm = 0.0,          // MEASURE
            positionZcm = 20.0,         // MEASURE
            yawDeg = 0.0,               // MEASURE: facing forward
            pitchDeg = 0.0,             // MEASURE
            rollDeg = 0.0,              // MEASURE
            imageToRobotYawDeg = 0.0,   // MEASURE (Test 3): wall moves backward when we go forward
            sceneDistanceCm = 150.0,    // MEASURE + TUNE (Test 3): typical robot-to-wall distance
            headingSign = -1.0,         // MEASURE (Test 2)
            translationTrust = 1.0,     // front camera carries translation
            headingTrust = 1.0,         // and also contributes heading
        ),
    )

    fun calibrationFor(id: CameraId): CameraCalibration = when (id) {
        CameraId.UP -> UP_CAMERA
        CameraId.FRONT -> FRONT_CAMERA
    }
}
