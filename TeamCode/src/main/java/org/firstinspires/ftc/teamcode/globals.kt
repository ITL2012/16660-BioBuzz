// i am used to in c++, where it is best practice to have all global variables in one file,
// so they are all easy to change and modify
package org.firstinspires.ftc.teamcode
import org.firstinspires.ftc.vision.opencv.ColorRange
import org.firstinspires.ftc.vision.opencv.ColorSpace
import org.opencv.core.Scalar

var DEBUG_MODE = true

const val DrivetrainFrontLeft = "frontleft"
const val DrivetrainFrontRight = "frontright"
const val DrivetrainBackLeft = "backleft"
const val DrivetrainBackRight = "backright"
const val LoaderServoName = "loader"

const val FlywheelLeftMotor = "flywheelleft"
const val FlywheelRightMotor = "flywheelright"
const val IntakeMotor = "intake"
const val IntakeGearRatio = 1.0

// Drivetrain geometry now DERIVES from the physical measurements in
// drivetrainspec.kt (DrivetrainSpec) — edit the measurements there, not here.
val WheelDiameterCm: Double = DrivetrainSpec.DRIVE_WHEEL_DIAMETER_CM
val WheelGearRatio: Double = DrivetrainSpec.DRIVE_GEAR_RATIO // spec-sheet value (NOT used in position math)
val WheelBaseCm: Double = DrivetrainSpec.WHEELBASE_CM
val TrackWidthCm: Double = DrivetrainSpec.TRACK_WIDTH_CM

// ================== MOTOR SPECS (goBILDA 5203-series Yellow Jackets) ==================
// Every motor below has a BUILT-IN quadrature encoder. The counts-per-rev (CPR)
// is what the NextFTC anglePerCount conversion uses, so encoder-based velocity
// and position control read TRUE wheel/motor speed. VERIFY each CPR against the
// spec sheet that ships in the goBILDA box before trusting closed-loop tuning.

// -- Drivetrain: 4x 117 RPM (5203-2402-0051), mecanum --
const val DRIVE_MOTOR_MAX_RPM = 312.0
// DERIVED from DrivetrainSpec (motor CPR x gearbox ratio) — no hand math.
val DRIVE_MOTOR_ENCODER_CPR = DrivetrainSpec.driveTicksPerWheelRev

// -- Flywheel: 2x 6000 RPM (5203-2402-0001), one per side of the ball --
const val FLYWHEEL_MOTOR_MAX_RPM = 6000.0
const val FLYWHEEL_MOTOR_ENCODER_CPR = 28.0 // 1:1 gearbox -> 28 CPR at output shaft

// -- Intake: single motor (set to your real motor's spec) --
const val INTAKE_MOTOR_MAX_RPM = 2500.0
const val INTAKE_MOTOR_ENCODER_CPR = 28.0

// ================== FLYWHEEL SHOOTER CALIBRATION (BIOBUZZ) ==================
// Landing distance (cm) of the ball for each flywheel RPM. MEASURE with the
// "Flywheel Calibration" OpMode + tools/flywheel_analyzer.py (see tools/README.md),
// then paste the analyzer's numbers into SHOOTER_CALIBRATION_DISTANCE_CM below,
// keeping the same order as SHOOTER_CALIBRATION_RPM (RPM descending).
// Until real distances are filled in (> 0), the shooter falls back to full speed.
val SHOOTER_CALIBRATION_RPM = doubleArrayOf(6000.0, 5000.0, 4000.0, 3000.0, 2000.0, 1000.0)
var SHOOTER_CALIBRATION_DISTANCE_CM = doubleArrayOf(
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // FILL IN: measured landing distance per RPM (cm)
)

// -- Flywheel priming: the 6000 RPM motors are torque-starved (1.47 kg.cm), so
// spin-up from a dead stop takes a while. Keep the wheels idling at PRIME_RPM
// so reaching any shooting speed is nearly instant. Set PRIME_ENABLED false to
// save battery / silence the wheels instead. --
const val FLYWHEEL_PRIME_ENABLED = true
const val FLYWHEEL_PRIME_RPM = 2000.0
// READY_RPM: always-on "warm" speed (2500 RPM) — instant spin-up to any shooting
// speed, draws ~1.5A vs ~4A at 6000 RPM. Used during autonomous/teleop when
// not actively shooting but wanting instant response.
const val FLYWHEEL_READY_RPM = 2500.0
// Fire only when BOTH wheels are within this many RPM of the commanded speed.
const val FLYWHEEL_SPEED_TOLERANCE_RPM = 150.0
// How long the intake runs to feed one ball into the spinning flywheel.
// (Legacy: firing now uses the LOADER below — this stays for manual/other uses.)
const val FLYWHEEL_FEED_TIME_MS = 1200.0

// ================== BALL LOADER (indexer between hopper and flywheel) ==================
// goBILDA dual-mode servo in POSITION mode, config name "loader". One full
// stroke meters exactly ONE ball into the flywheel (~360 deg at the output).
// Positions are normalized 0.0..1.0 (servo 0..300 deg through your gearing).
// MEASURE all four values on the real robot before trusting them.

const val LOADER_HOME_POSITION = 0.0      // servo position holding the gate closed
const val LOADER_STROKE_END_POSITION = 1.0 // one ball enters the flywheel here
const val LOADER_STROKE_MS = 800.0       // forward stroke duration incl. margin
const val LOADER_RETRACT_MS = 600.0      // return-to-home duration incl. margin

// ================== SHOT DETECTION (flywheel RPM-dip signature) ==================
// REV hub SERVO ports report no current, so the loader servo can't be the
// sensor — but the ball hitting the spinning wheels briefly loads them and the
// flywheel ENCODERS see a dip. Dip right after a stroke = a ball went through.
const val SHOT_DIP_THRESHOLD_RPM = 250.0  // dip below baseline that counts as a shot
const val SHOT_DETECT_WINDOW_MS = 900.0    // how long after a stroke we watch
const val SHOT_RECOVERY_TIMEOUT_MS = 2500.0 // wait for wheels to return to speed

// ================== NEURAL NETS (all on-robot: TFLite + pure-Kotlin MLP) ==================
// Everything runs inside the Robot Controller app itself — no co-processor, no
// network, lowest possible latency. Trained models live in the app's external
// storage (next to blackbox/): copy them over USB after training on the
// computer (see tools/README.md, the neural net section).
//
// Authority ladder (each component independently switchable):
//   OFF       not loaded — zero cost
//   ADVISORY  predictions go to telemetry + Black Box only (run this first,
//             for several matches, until the logged data proves the model)
//   REFEREE   the model can override the classical pipeline when confident
//   PRIMARY   the model can also ADD detections the classical pipeline missed
enum class NnMode { OFF, ADVISORY, REFEREE, PRIMARY }

// -- Ball/flower detector (tiny YOLO via TFLite; tools/auto_label + train_detector) --
val NN_DETECTOR_MODE = NnMode.ADVISORY   // flip to REFEREE when the data says so
const val NN_DETECT_HZ = 3.0             // inference rate — keep LOW: the hub CPU also runs vision
const val NN_DETECT_CONFIDENCE = 0.45    // detector score floor to accept a box
const val NN_DETECT_IOU = 0.45           // non-max suppression overlap cap
const val NN_REF_STALE_MS = 600.0         // a referee verdict is usable this long
const val NN_REF_MARGIN = 1.2            // blob center must sit inside a NN box x this margin
const val NN_MODEL_FILE = "nn/ball_detector.tflite"
val NN_LABELS = listOf("Pollen", "Nectar Blue", "Nectar Red", "Flower")

// -- Shot-outcome predictor (tiny MLP, pure Kotlin — no TFLite needed) --
val SHOT_PREDICTOR_MODE = NnMode.ADVISORY // flip to REFEREE to apply the learned nudge
const val SHOT_PREDICTOR_MIN_CONFIDENCE = 0.50 // below this predicted hit chance -> adjust
const val SHOT_PREDICTOR_MAX_ADJUST = 0.05     // bounded RPM correction: at most +/- 5%
const val SHOT_PREDICTOR_FILE = "nn/shot_predictor.json"

// ================== BLACK BOX (passive full-match data recorder) ==================
// Records EVERYTHING (state + both camera frame streams) to the Robot
// Controller's storage for post-match analysis on a computer. Fully passive:
// it never affects robot control, never transmits anything during a match,
// and only uses the two already-legal USB webcams. See docs/LEGALITY-REVIEW.md
// and blackbox.kt before using in official play.
const val BLACKBOX_ENABLED = true          // master switch
const val BLACKBOX_FRAME_CAPTURE_FPS = 5.0 // JPEG frames/sec saved PER camera

// ================== CAMERA FRAME CAPTURE (for post-match analysis) ==================
// The frames go into the session's SQLite file as BLOBs, so a match leaves
// the robot as ONE file. The rates and the budget below are the only knobs.
const val FRAME_CAPTURE_ENABLED = true           // master toggle for frame capture
const val BLACKBOX_MAX_SESSION_MB = 400.0   // storage budget per OpMode run
const val BLACKBOX_STATE_LOG_HZ = 10.0     // state lines per second

// ================== WORLD MODEL (persistent field memory) ==================
const val WM_TRACK_MATCH_CM = 40.0     // max distance to re-associate a sighting with a track
const val WM_BALL_MAX_AGE_SEC = 45.0    // how long an unseen ball stays remembered
const val WM_ROBOT_MAX_AGE_SEC = 6.0    // opponents are assumed to have moved by then
const val WM_CONFIDENCE_TAU_SEC = 10.0   // confidence decay time constant

// -- Ball pursuit upgrades (COLLECT phase; see BioBuzzAutonomous.collectBalls) --
const val WM_TRACK_EMA_ALPHA = 0.4        // track position smoothing on re-sight (0..1, higher = trust newer)
const val WM_VISIBLE_FRESH_SEC = 0.3      // a track re-sighted within this is "visible now"
const val DEAD_RECKON_MAX_AGE_SEC = 3.0   // chase an unseen ball's remembered spot for at most this long
const val CHASE_MIN_CONTOUR_AREA_PX = 350.0 // smaller/farther blobs are remembered but not chased
const val TARGET_SWITCH_CONFIRM_LOOPS = 3 // challenger must win this many consecutive planner picks to unseat a target
const val AVOID_ROBOT_BEARING_DEG = 15.0   // robot blob within this of the target bearing blocks pursuit
const val AVOID_ROBOT_STRAFE = 0.15        // sidestep power while dodging a blocking robot
const val CAPTURE_TURN_GAIN = 0.008        // turn power per degree of bearing (rotate toward the ball)
const val CAPTURE_TURN_MAX_POWER = 0.20    // cap on the turn-toward-ball term
const val SEARCH_SWEEP_TURN_POWER = 0.12   // in-place sweep when nothing is visible or remembered
const val AUTONOMY_PERIOD_SEC = 30.0 // full autonomous period — PhaseClock + planner clock frame

// -- Score-loop time budgeting (PhaseClock; see autonomy.kt) --
const val AUTONOMY_PARK_RESERVE_SEC = 3.0   // ALWAYS reserved for the park move
const val AUTONOMY_FIRE_RESERVE_SEC = 6.0   // reserved for spin-up + per-ball feeding (4 balls x 1.2s)
const val MIN_CYCLE_REMAINING_SEC = 14.0    // a second collect+align+fire cycle must fit inside this

// -- Pickup confirmation via intake motor current (a real ball-load signature) --
const val PICKUP_CURRENT_BUMP_A = 1.5       // MEASURE: current rise over baseline when a ball enters
const val PICKUP_BUMP_MIN_MS = 60.0         // bump must last at least this to count (debounce)
const val PICKUP_BUMP_MAX_MS = 450.0        // longer than this is a jam/obstruction, not a pickup

// -- Dodge anti-oscillation --
const val BLOCKED_GIVEUP_MS = 1500.0        // drop the committed target if blocked this long

// -- Neural detector hygiene (PRIMARY-mode ghost guard + model versioning) --
const val NN_MODEL_VERSION = "v0-untrained"  // bump whenever ball_detector.tflite is retrained — logged every match
const val NN_GHOST_RATE_LIMIT = 0.6          // PRIMARY additions beyond this EMA = demote to REFEREE (hallucinations)
const val NN_AGREE_EMA_ALPHA = 0.2          // smoothing for the HSV<->NN agreement statistics

// -- Teleop field-centric drive + pose hold (dead wheels / IMU powered) --
const val TELEOP_FIELD_CENTRIC_DEFAULT = false // start robot-centric; toggle with LEFT STICK BUTTON
const val TELEOP_POSE_HOLD_DEFAULT = true    // auto-hold position when sticks release; toggle RIGHT STICK BUTTON
const val TELEOP_INPUT_DEADZONE = 0.08       // sticks inside this count as "released"
const val TELEOP_HOLD_POSITION_KP = 0.06      // hold correction power per INCH of drift (RR pose is inches)
const val TELEOP_HOLD_HEADING_KP = 0.02      // hold turn power per degree of heading drift
const val TELEOP_HOLD_MAX_POWER = 0.25       // cap on any single hold correction

// ================== ANTI-RAMMING / PUSH-RESISTANCE ==================
// Push detection: when measured velocity diverges from commanded, an
// external force is shoving the robot. Engage maximum hold even during
// driver input (anchor mode).
const val PUSH_DETECTION_ENABLED = true
const val PUSH_VELOCITY_THRESHOLD = 0.35       // inches/sec divergence per axis to flag a push
const val PUSH_HEADING_THRESHOLD_DEG = 8.0     // heading rate divergence (deg/s) to flag a push
const val PUSH_DEBOUNCE_MS = 150.0             // must persist this long before engaging
const val PUSH_RELEASE_MS = 800.0              // hold anchor this long after push stops
const val ANCHOR_MAX_POWER = 0.85              // max corrective power during anchor (0.85 of 1.0)
const val ANCHOR_KP_POS = 0.15                 // position gain during anchor (per inch)
const val ANCHOR_KP_HEADING = 0.08             // heading gain during anchor (per deg)
const val ANCHOR_ENGAGE_GAMEPAD = true         // also allow manual engage via RIGHT_BUMPER+LB combo

// Vision fusion tightening on push: when pushed, trust vision corrections more
const val PUSH_FUSION_MAX_GAIN = 0.35          // override FUSION_MAX_GAIN during push
const val PUSH_FUSION_MAX_STEP_IN = 6.0        // override FUSION_MAX_STEP_IN during push
const val PUSH_FUSION_MAX_STEP_DEG = 4.0       // override FUSION_MAX_STEP_DEG during push

// Dodge: active opponent avoidance during teleop + autonomy
const val DODGE_ENABLED = true
const val DODGE_LOOKAHEAD_SEC = 1.2            // how far ahead to predict opponent path
const val DODGE_TRIGGER_DIST_CM = 120.0        // start dodging when opponent within this
const val DODGE_MIN_DIST_CM = 60.0             // emergency brake if opponent this close
const val DODGE_STRAFE_POWER = 0.45            // lateral dodge power
const val DODGE_TURN_POWER = 0.30              // turn-away power
const val DODGE_LATCH_MS = 800.0               // commit to dodge direction this long
const val DODGE_COOLDOWN_MS = 2000.0           // minimum time between dodges

// Autonomy stuck recovery: if position error accumulates, replan
const val AUTONOMY_STUCK_ENABLED = true
const val AUTONOMY_STUCK_POS_ERR_IN = 8.0      // position error (inches) to flag stuck
const val AUTONOMY_STUCK_HEADING_ERR_DEG = 15.0 // heading error (deg) to flag stuck
const val AUTONOMY_STUCK_DWELL_MS = 2500.0     // must persist this long before replan
const val AUTONOMY_STUCK_MAX_REPLANS = 2       // max replans per phase

// ================== PIN / CORNER ESCAPE ==================
// Pin detection: when commanded velocity is high but measured velocity
// is near zero (opponent pinning us). Triggers escape sequence.
const val PIN_DETECTION_ENABLED = true
const val PIN_VEL_RATIO_THRESHOLD = 0.15      // measured / commanded < this = stuck
const val PIN_DWELL_MS = 1000.0               // must persist this long
const val ESCAPE_BACKUP_POWER = 0.65          // reverse power to create gap
const val ESCAPE_BACKUP_DIST_IN = 12.0        // how far to back up (inches)
const val ESCAPE_ROTATE_DEG = 90.0            // then rotate this much
const val ESCAPE_RETRY_MAX = 3                // max escape attempts before give up
const val ESCAPE_COOLDOWN_MS = 3000.0         // minimum time between escapes

// ================== 3-WHEEL DEAD-WHEEL ODOMETRY (add the pods: ~zero-slip localization) ==================
// The Road Runner localizer uses these automatically WHEN all three motors exist
// in the hardware configuration; otherwise it falls back to the (slip-prone)
// mecanum drive-motor odometry. Bolt on any 3" style omni odometry pods and
// name the motors below in the Driver Hub config. All geometry (positions,
// angles, reversal flags, ticks per rev, wheel diameter) lives in
// DrivetrainSpec (drivetrainspec.kt) — measure there, the math happens here.
const val DEAD_WHEEL_PAR0_NAME = "par0"     // parallel wheel (forward rolling)
const val DEAD_WHEEL_PAR1_NAME = "par1"     // second parallel wheel
const val DEAD_WHEEL_PERP_NAME = "perp"     // perpendicular wheel

// ================== STRATEGIC PLANNER (lookahead brain) ==================
const val MATCH_TELEOP_SECONDS = 120.0      // BioBuzz driver-controlled period
const val PLANNER_COLLECT_HANDLE_SEC = 1.5  // intake time after reaching a ball
const val PLANNER_SCORE_TOTAL_SEC = 7.0     // align + spin-up + fire + recover
const val PLANNER_FLOWER_TOTAL_SEC = 10.0   // approach + pull + settle
const val PLANNER_PARK_SEC = 2.0            // final park move
const val PLANNER_OPPONENT_DISCOUNT = 0.5   // value multiplier when an opponent is closer
const val PLANNER_ENDGAME_MARGIN_SEC = 8.0  // don't start anything that can't finish

/**
 * How long a TYPICAL alignment takes, in seconds.
 *
 * The align phase has a hard cap of HiveScoringConfig.ALIGN_TIMEOUT_MS, but
 * charging the robot that worst case would make it refuse to score in the
 * endgame — a slow alignment is better than no score at all. The policy
 * budgets this typical figure; the phase deadline still cuts a slow
 * alignment short so the park is never missed.
 */
const val ALIGN_ESTIMATE_SEC = 4.0

/**
 * Roughly how long one more ball takes to gather: the drive to it plus the
 * intake handling. Used to work out whether there is still time to fill the
 * hopper AND score afterwards — charging the whole 9-second collection
 * phase instead would make the robot bank a half-full hopper early.
 */
const val COLLECT_PER_BALL_SEC = 2.5

// ================== MCTS PLANNER (deep lookahead) ==================
/** Playouts per decide() call. More playouts = deeper lookahead, more CPU. */
const val MCTS_PLAYOUTS = 128
/** Tree depth: how many actions ahead the search looks. */
const val MCTS_MAX_DEPTH = 4
/** Hard wall-clock budget for one decide() call (ms). Keeps the RC responsive. */
const val MCTS_TIME_BUDGET_MS = 60.0
/**
 * Progressive widening coefficient: a node may hold at most
 * `1 + floor(k * sqrt(visits))` children. Keeps the tree from exploding into
 * a combinatorial mess when there are many candidate actions, while still
 * letting a promising branch earn more branches.
 */
const val MCTS_PROGRESSIVE_WIDENING_K = 1.4
/**
 * Probability that a playout step follows the StrategicPlanner's pick
 * (policy guidance). The rest of the steps sample by value density so the
 * search still explores alternatives the heuristic dislikes.
 */
const val MCTS_HEURISTIC_WEIGHT = 0.65
/** Temperature of the value-density softmax used for exploratory playout steps. */
const val MCTS_SOFTMAX_TEMPERATURE = 2.0

// ================== OPPONENT PARTICLE FILTER ==================
/** Particles per tracked opponent. More = better occlusion survival, more CPU. */
const val OPP_PARTICLE_COUNT = 40
/** Position process noise per sqrt(second), cm. */
const val OPP_PARTICLE_POS_NOISE_CM = 8.0
/** Velocity process noise per sqrt(second), cm/s. */
const val OPP_PARTICLE_VEL_NOISE_CM_PER_SEC = 15.0
/** Measurement noise of an opponent detection, cm (sigma of the Gaussian). */
const val OPP_PARTICLE_MEAS_NOISE_CM = 30.0
/** Resample when the effective sample size falls below this fraction of N. */
const val OPP_PARTICLE_RESAMPLE_ESS_RATIO = 0.5
/** Fraction of particles re-injected (spread) after a resample, for diversity. */
const val OPP_PARTICLE_REINJECT_RATIO = 0.25
/** EMA weight of the particle-filter position when fusing into a track. */
const val OPP_PARTICLE_FUSE_ALPHA = 0.6
/**
 * How much of the measured velocity is blended into each particle's own
 * velocity. High enough that the cloud follows the opponent between
 * frames, low enough that the process noise keeps a spread around it.
 */
const val OPP_PARTICLE_VEL_HINT_ALPHA = 0.7

// ================== FLOWER CLUSTERING + DEPLETION ==================
/** Lower clamp for the adaptive clustering epsilon, cm. */
const val FLOWER_CLUSTER_EPS_MIN_CM = 25.0
/** Upper clamp for the adaptive clustering epsilon, cm. */
const val FLOWER_CLUSTER_EPS_MAX_CM = 90.0
/**
 * Multiplier on the mean nearest-neighbour distance of the visible flowers.
 * Sparse field -> big epsilon (group loose flowers); dense patch -> small
 * epsilon (don't merge separate patches).
 */
const val FLOWER_CLUSTER_EPS_KNN_FACTOR = 1.8
/** How often the cluster set is rebuilt, seconds. */
const val FLOWER_CLUSTER_REBUILD_SEC = 1.0
/** Base cooldown after a flower (or cluster) is pulled, seconds. */
const val FLOWER_COOLDOWN_BASE_SEC = 30.0
/** Extra cooldown per previous pull of the same flower/cluster, seconds. */
const val FLOWER_COOLDOWN_PER_PULL_SEC = 10.0
/** Regrowth time after the cooldown expires, seconds. */
const val FLOWER_REGEN_SEC = 15.0
/**
 * Radius (cm) within which two nectar pulls count as the SAME spot on the
 * field. Pulls inside this radius merge into one depleted patch, so the
 * cooldown grows with every pull from the same flower bed instead of being
 * reset by each new pull.
 */
const val FLOWER_PATCH_MERGE_CM = 45.0

// ================== DYNAMIC RESERVES (PhaseClock) ==================
/** Fixed spin-up + alignment buffer in the fire reserve, seconds. */
const val RESERVE_FIRE_BASE_SEC = 3.0
/** Feed time per ball in the fire reserve, seconds. */
const val RESERVE_FIRE_PER_BALL_SEC = 0.8
/** Clamp on the computed fire reserve, seconds. */
const val RESERVE_FIRE_MAX_SEC = 12.0
/** Clamp on the computed park reserve, seconds. */
const val RESERVE_PARK_MAX_SEC = 8.0
/** Extra seconds reserved for parking when the robot is far from a wall. */
const val RESERVE_PARK_DISTANT_SEC = 4.0
/** Distance (cm) beyond which the robot is considered "far" from any wall. */
const val RESERVE_PARK_DISTANT_CM = 120.0

// ================== PHASE SCHEDULER ==================
/**
 * A higher-priority phase may preempt the running one, but only by this
 * much priority — avoids thrashing between two nearly-equal phases.
 */
const val SCHEDULER_PREEMPT_PRIORITY_MARGIN = 10
/**
 * A running phase is only preempted when at least this much of its budget
 * is still unspent — otherwise we let it finish (finishing is cheap, the
 * switch is not).
 */
const val SCHEDULER_PREEMPT_MIN_REMAINING_SEC = 1.5

// ================== REFLEXES (below-the-planner safety) ==================
const val INTAKE_JAM_CURRENT_A = 3.5      // amps above this (while spinning) = jam suspected
const val INTAKE_JAM_SUSTAIN_MS = 300.0   // current must stay high this long
const val INTAKE_COUGH_MS = 450.0         // reverse pulse to clear a jam
const val INTAKE_MAX_COUGHS_PER_JAM = 3  // give up after this many reverse pulses
const val TILT_GUARD_DEG = 28.0           // cut drive power beyond this pitch/roll

// Shadow mode: the planner watches human teleop and logs what it WOULD do.
const val SHADOW_PLANNER_ENABLED = true

// Strategic/Pathfinding Constants (BIOBUZZ)
const val NECTAR_SCORE_MULTIPLIER = 2.0 // Nectar is worth more points than Pollen
const val ROBOT_SPEED_CM_PER_SEC = 100.0 // Placeholder: Measure actual robot speed
/** Seconds of travel budgeted per cm when estimating a time reserve. */
const val RESERVE_TRAVEL_SEC_PER_CM = 1.0 / ROBOT_SPEED_CM_PER_SEC
const val FLYWHEEL_SPINUP_SEC = 2.0 // Placeholder: informational only; spin-up runs in parallel with travel
const val FLYWHEEL_PRESPIN_BALL_COUNT = 2 // Start warming the flywheel once we hold this many balls

// ================== BIOBUZZ GAME LOGIC (tune these between matches) ==================
// Everything you'd plausibly change on game day lives HERE or is indexed below.
// Algorithm-internal tuning lives in dedicated configs, all listed so nothing
// is hard to find from this file:
//   * Shooting range window / aim gains  -> hivescoring.kt  HiveScoringConfig
//   * Vision tracking/fusion thresholds   -> vision/VisionTuning.kt
//   * Camera calibrations (MEASURE)       -> vision/VisionConfig.kt
//   * Road Runner drive model/gains      -> roadrunner/MecanumDrive.kt PARAMS
//   * AprilTag test targets + camera names-> this file (see camera section above)

// -- Ball capacity (single source of truth; the WorldModel picker + planner use it) --
const val MAX_BALLS_HELD = 4            // MEASURE: how many balls the robot can store

// -- Ball point values (keep in sync with NECTAR_SCORE_MULTIPLIER) --
const val POLLEN_VALUE = 1.0
const val NECTAR_VALUE = 2.0

// -- Ball targeting priority (autonomous collection decisions) --
// Normal slots: utility = value * PRIORITY_VALUE_WEIGHT
//                        / (1 + distanceMeters * PRIORITY_DISTANCE_WEIGHT)
// Raise VALUE_WEIGHT to chase nectar harder; raise DISTANCE_WEIGHT to prefer near balls.
const val PRIORITY_VALUE_WEIGHT = 1.0
const val PRIORITY_DISTANCE_WEIGHT = 1.0
// With exactly ONE slot left, take the highest-value ball outright (a 2x ball
// in the last slot beats a 1x ball even if somewhat farther).
const val LAST_SLOT_PREFER_HIGHEST_VALUE = true

// -- Autonomous collection behavior --
const val AUTONOMOUS_COLLECT_TIMEOUT_MS = 9000.0 // hard cap on the collection phase
const val PICKUP_CONFIRM_CM = 25.0   // ball closer than this can count as picked up
const val PICKUP_SETTLE_MS = 400.0    // must stay that close this long to confirm
const val MIN_PICKUP_ENCODER_TICKS = 50L // encoder ticks per ball (MEASURE: ticks added per ball)

// ================== FLOWER PULL-OUT — CUSTOMIZE FOR YOUR MECHANISM ==================
const val FLOWER_PULL_ENABLED = true         // autonomous tries flowers at all
const val FLOWER_APPROACH_CM = 25.0          // MEASURE: standoff taken before pulling
const val FLOWER_PULL_TIMEOUT_MS = 6000.0    // phase cap (approach + pull)
const val FLOWER_SETTLE_AFTER_PULL_MS = 600L // intake time to swallow the released ball

/**
 * ==== HOW THE ROBOT PULLS A BALL OUT OF A FLOWER — EDIT THIS ====
 *
 * Autonomous drives to FLOWER_APPROACH_CM of a flower, stops, and calls
 * this routine. Replace the body with the real mechanism sequence for the
 * final design (servo sweep, arm motion, winch, etc.). It runs ONCE per
 * flower stop, on the OpMode thread, so Thread.sleep / blocking waits are OK
 * in AUTONOMOUS (a phase loop that fully stops first).
 *
 * SelfPlay also calls it, but there it runs inside the live control loop: it
 * is latched to fire at most ONCE per flower approach, and while it runs the
 * deadman is not polled — so keep the routine SHORT (well under ~0.5 s), or
 * restructure it as a non-blocking state machine when the real mechanism
 * exists.
 *
 * The active OpMode passes in its own Intake/Loader instances (there are no
 * hardware singletons anymore — see the note in TestOpModes.kt).
 *
 * Default placeholder: reverse-pulse the intake (assumes pulling = backing
 * the ball out with the intake until a real mechanism exists).
 */
var flowerPullRoutine: (Intake, Loader) -> Unit = { intake, _ ->
    intake.spinUp(-1.0)
    // The NextFTC event loop must be polled or the spinUp setpoint never
    // reaches the motor during the blind sleep below.
    ControlFrame.forcePoll()
    try {
        Thread.sleep(1000)
    } catch (_: InterruptedException) {
        Thread.currentThread().interrupt()
    }
    intake.stop()
    ControlFrame.forcePoll()
}

/** Type of ball collected, for the ball counter. UNKNOWN = source cannot identify type (e.g. beam-break sensor). */
enum class BallType { POLLEN, NECTAR, UNKNOWN }

// some experimental camera stuff

const val GROUND_CAM_NAME = "Webcam Ground"
const val HIGH_CAM_NAME = "Webcam High"

// --- Hybrid visual localization (two cameras with intentionally different views) ---
// The UP camera (ceiling: rafters, trusses, lights, beams) and the FRONT camera
// (field walls, perimeter, posts, corners, rigid structures) feed ONE pose estimator.
// These default to the webcams already present in the hardware configuration;
// change the names here if dedicated cameras are added.
const val UP_CAM_NAME = HIGH_CAM_NAME      // MEASURE: camera angled upward at the ceiling
const val FRONT_CAM_NAME = GROUND_CAM_NAME // MEASURE: camera facing forward at field structures

// MEASURE: the IMU device name in the hardware configuration ("imu" is the REV Hub default).
const val RR_IMU_NAME = "imu"
// HIGH (UP) camera resolution — must be a mode the Logitech C270 actually
// supports (640x480, 800x448, 1280x720) and must match VisionTuning's
// UP_CAM_WIDTH/HEIGHT, which the UP-camera intrinsics (VisionConfig.UP_CAMERA)
// are calibrated for. Calibrate real values with the "Camera Calibration (C270)" OpMode.
const val CAMERA_WIDTH_PIXELS = 800
const val CAMERA_HEIGHT_PIXELS = 448
const val GROUND_CAM_WIDTH = 640
const val GROUND_CAM_HEIGHT = 480
const val C270_FOCAL_LENGTH_PIXELS = 543.0
const val C270_HORIZONTAL_FOV_DEGREES = 47.0
const val POLLEN_PHYSICAL_WIDTH_MM = 71.12
const val NECTAR_PHYSICAL_WIDTH_MM = 91.44
const val APRILTAG_PHYSICAL_WIDTH_MM = 82.55 // 3.25 inches
const val FLOWER_PHYSICAL_WIDTH_MM = 101.5 // 4 inches (opening)
const val ROBOT_PHYSICAL_WIDTH_MM = 450.0 // ~18 inch robot footprint, for range estimation
// AprilTag IDs for BIOBUZZ Hives
val RED_HIVE_TAGS = listOf(30, 31, 32, 33, 34, 35, 36, 37)
val BLUE_HIVE_TAGS = listOf(38, 39, 40, 41, 42, 43, 44, 45)

// Official BIOBUZZ hive AprilTag CLUSTERS (SDK 12.0 game tag library).
// Each hive is a 4-tag CLUSTER whose origin is the CENTER OF THE CELL OPENING —
// so the cluster pose IS the scoring aim point. Match by shortName because the
// cluster member-ID accessors are not public in the SDK.
//   RED SCORING: tags 30-33    RED AUDIENCE: tags 34-37
//   BLUE AUDIENCE: tags 38-41  BLUE SCORING: tags 42-45
const val HIVE_CLUSTER_RED_SCORING_NAME = "RED SCORING"
const val HIVE_CLUSTER_RED_AUDIENCE_NAME = "RED AUDIENCE"
const val HIVE_CLUSTER_BLUE_AUDIENCE_NAME = "BLUE AUDIENCE"
const val HIVE_CLUSTER_BLUE_SCORING_NAME = "BLUE SCORING"
val HIVE_CLUSTER_NAMES = setOf(
    HIVE_CLUSTER_RED_SCORING_NAME,
    HIVE_CLUSTER_RED_AUDIENCE_NAME,
    HIVE_CLUSTER_BLUE_AUDIENCE_NAME,
    HIVE_CLUSTER_BLUE_SCORING_NAME,
)

// ================== TEAM (ALLIANCE) SELECTION ==================
/**
 * Which alliance we are playing on. Both match OpModes let the drivers pick
 * this live during INIT (D-pad LEFT = RED, D-pad RIGHT = BLUE); the value
 * below is only the fallback when nobody touches the gamepad.
 *
 * Everything that targets hives (HiveScoring, VisionSystem) filters AprilTags
 * through teamTagIds() / teamClusterNames(), so the robot can never align on
 * or fire into the opposing alliance's hive.
 */
enum class TeamColor { RED, BLUE }

var SELECTED_TEAM: TeamColor = TeamColor.RED

/** AprilTag IDs of ONE alliance's hives (single-tag fallback path). */
fun teamTagIds(team: TeamColor): List<Int> = when (team) {
    TeamColor.RED -> RED_HIVE_TAGS
    TeamColor.BLUE -> BLUE_HIVE_TAGS
}

/** SDK 12.0 hive-cluster short-names of ONE alliance's hives (both sides). */
fun teamClusterNames(team: TeamColor): Set<String> = when (team) {
    TeamColor.RED -> setOf(HIVE_CLUSTER_RED_SCORING_NAME, HIVE_CLUSTER_RED_AUDIENCE_NAME)
    TeamColor.BLUE -> setOf(HIVE_CLUSTER_BLUE_SCORING_NAME, HIVE_CLUSTER_BLUE_AUDIENCE_NAME)
}

// HSV Ranges for BIOBUZZ (OpenCV hue H runs 0-180).
// The bands are separated by DEAD BANDS so venue lighting can't bleed one
// game piece into another:  red 0-12  |  pollen 14-35  |  flower 45-80.
// Re-tune on-site with the "Vision Test OpMode" if detections misfire.
val POLLEN_HSV_RANGE = ColorRange(
    ColorSpace.HSV,
    Scalar(14.0, 100.0, 100.0),
    Scalar(35.0, 255.0, 255.0)
)
val NECTAR_BLUE_HSV_RANGE = ColorRange(
    ColorSpace.HSV,
    Scalar(100.0, 100.0, 100.0),
    Scalar(140.0, 255.0, 255.0)
)
// OpenCV hue wraps at 180, so true red sits near BOTH H=0 and H=180.
// ColorRange cannot express a wrapping band, so red nectar needs a second
// range + processor with the SAME "Nectar Red" label (see BallLocator /
// VisionSystem) — downstream scoring/priority logic is unchanged.
val NECTAR_RED_HSV_RANGE = ColorRange(
    ColorSpace.HSV,
    Scalar(0.0, 100.0, 100.0),
    Scalar(12.0, 255.0, 255.0)
)
val NECTAR_RED_WRAP_HSV_RANGE = ColorRange(
    ColorSpace.HSV,
    Scalar(168.0, 100.0, 100.0),
    Scalar(180.0, 255.0, 255.0)
)
val FLOWER_GREEN_HSV_RANGE = ColorRange(
    ColorSpace.HSV,
    Scalar(45.0, 50.0, 50.0),
    Scalar(80.0, 255.0, 255.0)
)
// Robots: large dark shapes (black plastic, shadows). Loose V/S bounds catch most lighting.
val ROBOT_DARK_HSV_RANGE = ColorRange(
    ColorSpace.HSV,
    Scalar(0.0, 0.0, 0.0),
    Scalar(180.0, 120.0, 90.0)
)

// Minimum contour area (px^2 at 640x480) for a blob to count as a robot, not noise.
const val MIN_ROBOT_CONTOUR_AREA = 4000.0
