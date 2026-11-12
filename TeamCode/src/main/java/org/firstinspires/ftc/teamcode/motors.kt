package org.firstinspires.ftc.teamcode
import dev.nextftc.hardware.actuators.NextMotor
import dev.nextftc.hardware.actuators.NextServo
import dev.nextftc.robot.Mechanism
import dev.nextftc.units.RotationsPerMinute
import dev.nextftc.units.radians
import dev.nextftc.units.rpm
import kotlin.math.roundToInt
import kotlin.math.sqrt
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

/**
 * Shared setup for one kind of goBILDA 5203-series Yellow Jacket (specs in
 * globals.kt). Sets the encoder counts->angle conversion (anglePerCount) so
 * closed-loop modes read TRUE wheel speed/position, plus sane starting gains:
 *
 *   * kV — throttle per (rad/s) of wheel speed; sized so kV * top speed ~= 1.0
 *   * kP — extra throttle per (rad/s) of velocity error (starts equal to kV)
 *   * kS — static-friction kick
 *
 * These are STARTING POINTS: tune them live (FTC Dashboard) or edit here if
 * the drivetrain lags or oscillates.
 */

/** Base position-PID kP used by NextFTC position moves; [Drivetrain.setSpeed] scales it. */
private const val POSITION_KP = 8.0

private fun nextYellowJacket(
    name: String,
    maxRpm: Double,
    encoderCpr: Double,
    configure: NextMotor.() -> Unit = {},
): NextMotor = NextMotor(
    name,
    anglePerCount = (2.0 * PI / encoderCpr).radians,
).apply {
    configure()
    // Velocity setpoints are commanded in RPM (see setVelocitySetpoint callers),
    // and NextFTC's feedforward/PID operate on the raw setpoint magnitude, so
    // gains must be per-RPM, not per-(rad/s).
    val velocityKv = 1.0 / maxRpm      // full throttle at the motor's top speed
    velocityConstants.apply {
        kV = velocityKv
        kP = velocityKv               // start with P ~= V; raise if it lags
        kI = 0.0
        kS = 0.03                     // static-friction compensation
    }
    positionConstants.apply {
        kP = POSITION_KP              // throttle per radian of position error (TUNE)
        kD = 0.0
        kV = 1.0 / maxRpm
    }
}

class Drivetrain : Mechanism {

    val FrontLeft = nextYellowJacket(DrivetrainFrontLeft, DRIVE_MOTOR_MAX_RPM, DRIVE_MOTOR_ENCODER_CPR) {
        direction = NextMotor.Direction.FORWARD // use FORWARD or REVERSE
        zeroPowerBehavior = NextMotor.ZeroPowerBehavior.BRAKE // use BRAKE or FLOAT
    }

    val FrontRight = nextYellowJacket(DrivetrainFrontRight, DRIVE_MOTOR_MAX_RPM, DRIVE_MOTOR_ENCODER_CPR) {
        direction = NextMotor.Direction.FORWARD
        zeroPowerBehavior = NextMotor.ZeroPowerBehavior.BRAKE // use BRAKE or FLOAT
    }

    val BackLeft = nextYellowJacket(DrivetrainBackLeft, DRIVE_MOTOR_MAX_RPM, DRIVE_MOTOR_ENCODER_CPR) {
        direction = NextMotor.Direction.FORWARD
        zeroPowerBehavior = NextMotor.ZeroPowerBehavior.BRAKE // use BRAKE or FLOAT
    }

    val BackRight = nextYellowJacket(DrivetrainBackRight, DRIVE_MOTOR_MAX_RPM, DRIVE_MOTOR_ENCODER_CPR) {
        direction = NextMotor.Direction.FORWARD
        zeroPowerBehavior = NextMotor.ZeroPowerBehavior.BRAKE // use BRAKE or FLOAT
    }

    /**
     * cm -> wheel ticks / wheel radians. Delegates to DrivetrainSpec so the
     * units chain (encoder ticks = wheel radians = NextFTC position units)
     * has exactly ONE definition. The old WheelGearRatio multiplier is GONE:
     * the gearbox is already folded into driveTicksPerWheelRev, and NextFTC
     * positions are in WHEEL radians — motor radians would overshoot by the
     * gear ratio.
     */
    fun cmToTicks(cm: Double): Int = DrivetrainSpec.cmToWheelTicks(cm)
    fun cmToRad(cm: Double): Double = DrivetrainSpec.cmToWheelRad(cm)

    /**
     * Commands each wheel to rotate by the given deltas (radians), measured
     * FROM ITS CURRENT POSITION. NextFTC position setpoints are ABSOLUTE in
     * the accumulated-encoder frame — issuing them without the current
     * position would target "N radians since hub boot", which is only a
     * relative move on a freshly zeroed encoder and never twice in a row.
     */
    private fun moveBy(dFl: Double, dFr: Double, dBl: Double, dBr: Double) {
        FrontLeft.setPositionSetpoint(FrontLeft.encoderPosition + dFl.radians)
        FrontRight.setPositionSetpoint(FrontRight.encoderPosition + dFr.radians)
        BackLeft.setPositionSetpoint(BackLeft.encoderPosition + dBl.radians)
        BackRight.setPositionSetpoint(BackRight.encoderPosition + dBr.radians)
    }

    /**
     * Moves the robot FORWARD [cm] centimeters relative to its current spot
     * (mecanum forward = all wheels forward). Re-issuing extends the move.
     * [speed] scales the position controller's kP (see [setSpeed]).
     */
    fun driveForward(cm: Double, speed: Double = 1.0) {
        val rad: Double = cmToRad(cm)
        setSpeed(speed)
        moveBy(rad, rad, rad, rad)
    }
    fun driveBackward(cm: Double, speed: Double = 1.0) {driveForward(-cm, speed)}

    fun driveRight(cm: Double, speed: Double = 1.0) {
        val wheelCm = cm * sqrt(2.0)
        val rad = cmToRad(wheelCm)

        setSpeed(speed)

        moveBy(rad, -rad, -rad, rad)
    }

    fun driveLeft(cm: Double, speed: Double = 1.0) {
        val wheelCm = cm * sqrt(2.0)
        val rad = cmToRad(wheelCm)
        setSpeed(speed)
        moveBy(-rad, rad, rad, -rad)
    }

    fun turn(degrees: Double, speed: Double = 1.0) {
        val wheelRad = DrivetrainSpec.turnToWheelRad(degrees)

        setSpeed(speed)

        moveBy(wheelRad, -wheelRad, wheelRad, -wheelRad)
    }

    /**
     * Moves the robot at a specific angle relative to its current orientation for a given distance.
     * @param cm The distance to move in centimeters.
     * @param degrees The angle at which to move (0 is forward, 90 is right, 180 is backward, 270 is left).
     * @param speed The speed multiplier (0.0 to 1.0).
     */
    fun driveAtAngle(cm: Double, degrees: Double, speed: Double = 1.0) {
        val radAngle = degrees * PI / 180.0
        val forward = cm * cos(radAngle)
        val strafe = cm * sin(radAngle)

        val strafeMultiplier = sqrt(2.0)

        val flWheel = forward + strafe * strafeMultiplier
        val frWheel = forward - strafe * strafeMultiplier
        val blWheel = forward - strafe * strafeMultiplier
        val brWheel = forward + strafe * strafeMultiplier

        setSpeed(speed)

        moveBy(
            cmToRad(flWheel), cmToRad(frWheel),
            cmToRad(blWheel), cmToRad(brWheel),
        )
    }

    fun driveTeleOp(forward: Double, strafe: Double, turn: Double) {
        var frontLeft = forward + strafe + turn
        var frontRight = forward - strafe - turn
        var backLeft = forward - strafe + turn
        var backRight = forward + strafe - turn

        // Normalize so none exceeds 1.0
        val max = maxOf(
            1.0,
            kotlin.math.abs(frontLeft),
            kotlin.math.abs(frontRight),
            kotlin.math.abs(backLeft),
            kotlin.math.abs(backRight)
        )

        frontLeft /= max
        frontRight /= max
        backLeft /= max
        backRight /= max

        FrontLeft.setVelocitySetpoint((frontLeft * DRIVE_MOTOR_MAX_RPM).rpm)
        FrontRight.setVelocitySetpoint((frontRight * DRIVE_MOTOR_MAX_RPM).rpm)
        BackLeft.setVelocitySetpoint((backLeft * DRIVE_MOTOR_MAX_RPM).rpm)
        BackRight.setVelocitySetpoint((backRight * DRIVE_MOTOR_MAX_RPM).rpm)
    }


    /**
     * Speed scale for the NEXT position move (driveForward/turn/driveAtAngle).
     *
     * NextFTC position control has no separate target-velocity knob — a
     * position move's speed comes from the position PID's kP — so "speed"
     * here scales kP: 1.0 = full gain ([POSITION_KP], the tuned value),
     * 0.25 = a gentle quarter-gain crawl. Call it BEFORE the move (all move
     * helpers call it for you). Velocity control ([driveTeleOp]) is unaffected.
     *
     * (The old body was commented out, which silently ignored every `speed`
     * argument in this file — DriveOnly's 0.25 nudge, the pursuit moves'
     * 0.5/0.8 approach speeds, the scripted test-bench moves.)
     */
    fun setSpeed(speed: Double = 1.0) {
        val kp = POSITION_KP * speed.coerceIn(0.0, 1.0)
        FrontLeft.positionConstants.kP = kp
        FrontRight.positionConstants.kP = kp
        BackLeft.positionConstants.kP = kp
        BackRight.positionConstants.kP = kp
    }


    /**
     * Actually stops the drivetrain: switches every motor from position
     * control to a zero-velocity command (the setpoint persists otherwise —
     * the robot would keep driving to its last position target).
     */
    fun flushMotors(noCapSpeed: Boolean = true) {
        if (noCapSpeed) {
            FrontLeft.setVelocitySetpoint(0.0.rpm)
            FrontRight.setVelocitySetpoint(0.0.rpm)
            BackLeft.setVelocitySetpoint(0.0.rpm)
            BackRight.setVelocitySetpoint(0.0.rpm)
        }
    }
}
class Intake(private val hasMotor: Boolean = true) : Mechanism {

    /**
     * Null when the intake motor isn't in the configuration — NextMotor
     * self-registers with the NextFTC event loop at construction, so a motor
     * must never be constructed for a missing device (background-thread
     * crash). Probe-gated OpModes pass hasMotor from HardwareProbe.
     */
    val intakeMotor: NextMotor? =
        if (hasMotor) nextYellowJacket(IntakeMotor, INTAKE_MOTOR_MAX_RPM, INTAKE_MOTOR_ENCODER_CPR) {
            direction = NextMotor.Direction.FORWARD // use FORWARD or REVERSE
            zeroPowerBehavior = NextMotor.ZeroPowerBehavior.FLOAT // use BRAKE or FLOAT
        } else null

    val available: Boolean get() = hasMotor

    // Encoder tracking for multi-ball pickup confirmation
    private var encoderAtPickupStart = 0L
    var lastPickupEncoderDelta = 0L
    private var pickupsSinceReset = 0

    /** Captures the current encoder position as the baseline for the next pickup. */
    fun markPickupStart() {
        encoderAtPickupStart = Math.round(intakeMotor?.encoderPosition as? Double ?: 0.0).toLong()
    }

    /** Returns encoder ticks accumulated since markPickupStart(). */
    fun encoderDeltaSincePickupStart(): Long {
        val current = Math.round(intakeMotor?.encoderPosition as? Double ?: 0.0).toLong()
        return current - encoderAtPickupStart
    }

    /** Confirms a pickup via encoder delta. Resets counter and returns true if delta exceeds threshold. */
    fun confirmPickupByEncoder(minTicks: Long = 50): Boolean {
        val delta = encoderDeltaSincePickupStart()
        if (delta >= minTicks) {
            pickupsSinceReset++
            lastPickupEncoderDelta = delta
            encoderAtPickupStart = Math.round(intakeMotor?.encoderPosition as? Double ?: 0.0).toLong() // reset baseline
            return true
        }
        return false
    }

    /** Total confirmed pickups since last reset. */
    val confirmedPickups: Int get() = pickupsSinceReset

    fun spinUp(speed: Double = 1.0) {
        // NOTE: full range -1..1 — reverse (gamepad B in teleop) is a real
        // reverse here; the old code clamped negatives to a stop.
        val velocity = speed.coerceIn(-1.0, 1.0) * INTAKE_MOTOR_MAX_RPM
        intakeMotor?.setVelocitySetpoint(velocity.rpm)
    }
    fun stop(breakMotor: Boolean = false) {
        intakeMotor?.let { m ->
            m.zeroPowerBehavior =
                if (breakMotor)
                    NextMotor.ZeroPowerBehavior.BRAKE
                else
                    NextMotor.ZeroPowerBehavior.FLOAT
            m.setVelocitySetpoint(0.0.rpm)
        }
    }

    /** Present motor current draw in amps (0.0 when missing/unreadable). */
    val currentAmps: Double
        get() = try {
            intakeMotor?.current?.magnitude ?: 0.0
        } catch (_: Exception) {
            0.0
        }
}

/**
 * Detects a ball entering the intake by its CURRENT signature: a real load is
 * a brief current rise above the running baseline (and back down) — unlike a
 * jam, which stays high (IntakeJamGuard owns that case). Used by the COLLECT
 * phase as physical pickup confirmation, so ball counting no longer relies on
 * settle/vanish inference alone.
 *
 * MEASURE PICKUP_CURRENT_BUMP_A on the real robot: with the intake spinning
 * empty, read telemetry baseline current; then feed a ball and read the peak.
 */
class PickupCurrentSensor(private val intake: Intake) {

    private var baselineAmps = 0.5
    private var highSinceNanos = 0L

    /**
     * Call every loop. [spinning] = whether the intake is being driven right
     * now (sensor idles otherwise). Returns true exactly once per completed
     * ball-load bump.
     */
    fun update(spinning: Boolean): Boolean {
        val now = System.nanoTime()
        if (!spinning || !intake.available) {
            highSinceNanos = 0L
            return false
        }

        val current = intake.currentAmps
        return if (current >= baselineAmps + PICKUP_CURRENT_BUMP_A) {
            if (highSinceNanos == 0L) highSinceNanos = now
            false
        } else {
            // Slowly track the empty-running baseline when current is low.
            baselineAmps += 0.05 * (current - baselineAmps)
            if (highSinceNanos == 0L) return false
            val bumpMs = (now - highSinceNanos) / 1e6
            highSinceNanos = 0L
            // A rise that lasted between min and max = one ball went in.
            bumpMs in PICKUP_BUMP_MIN_MS..PICKUP_BUMP_MAX_MS
        }
    }
}

/**
 * DUAL 6000 RPM flywheel: one 5203-series Yellow Jacket on each side of the
 * ball, spinning OPPOSITE physical directions (left FORWARD, right REVERSE)
 * so the ball is squeezed straight through. Both wheels run the same commanded
 * RPM, and encoder feedback ([isAtSpeed]) verifies they actually reached it
 * before anything fires.
 *
 * The 6000 RPM motors only make 1.47 kg.cm of torque, so spin-up from a dead
 * stop is slow — keep the wheels PRIMED ([prime]) whenever we hold balls, and
 * pre-spin to the speed the hive distance calls for (see [ShooterModel],
 * driven by [HiveScoring.shootingRpmFor]) while approaching.
 * PARTIAL-ROBOT SUPPORT: pass hasLeft/hasRight from a HardwareProbe — a
 * motor is only CONSTRUCTED for a device that exists in the configuration
 * (NextMotor self-registers with the shared NextFTC event loop, so
 * constructing one for a missing device crashes on a background thread). A
 * one-wheel flywheel still spins, reads its encoder, and passes the RPM-dip
 * shot check — perfect for bench testing; it just can't launch a ball well.
 */
class FlyWheel(
    private val hasLeft: Boolean = true,
    private val hasRight: Boolean = true,
) : Mechanism {

    /** True when at least one wheel exists. */
    val anyMotor: Boolean get() = hasLeft || hasRight

    private fun motor(name: String, reverse: Boolean): NextMotor? =
        if (if (reverse) hasRight else hasLeft)
            nextYellowJacket(name, FLYWHEEL_MOTOR_MAX_RPM, FLYWHEEL_MOTOR_ENCODER_CPR).apply {
                direction = if (reverse) NextMotor.Direction.REVERSE else NextMotor.Direction.FORWARD
                zeroPowerBehavior = NextMotor.ZeroPowerBehavior.FLOAT
            }
        else null

    private val flyWheelLeft = motor(FlywheelLeftMotor, reverse = false)
    private val flyWheelRight = motor(FlywheelRightMotor, reverse = true)

    /** Holds whichever wheels exist at [rpm], clamped to the motor's real top speed. */
    fun spinToRpm(rpm: Double) {
        val target = rpm.coerceIn(0.0, FLYWHEEL_MOTOR_MAX_RPM)
        flyWheelLeft?.setVelocitySetpoint(target.rpm)
        flyWheelRight?.setVelocitySetpoint(target.rpm)
    }

    /** Legacy power-style control: 1.0 = full 6000 RPM, 0 = stopped. */
    fun spinUp(speed: Double = 1.0) {
        spinToRpm(speed.coerceIn(-1.0, 1.0) * FLYWHEEL_MOTOR_MAX_RPM)
    }

    /** Idles whichever wheels exist at [FLYWHEEL_READY_RPM] (2500 RPM) — instant spin-up
 *  to any shooting speed, draws ~1.5A vs ~4A at 6000 RPM. Used as "always ready" state. */
    fun ready() {
        spinToRpm(FLYWHEEL_READY_RPM)
    }

    /** Legacy: idles at PRIME_RPM (2000 RPM) — kept for backward compatibility. */
    @Deprecated("Use ready() instead", ReplaceWith("ready()"))
    fun prime() {
        spinToRpm(FLYWHEEL_PRIME_RPM)
    }

    fun stop(breakMotor: Boolean = false) {
        val zpb = if (breakMotor)
            NextMotor.ZeroPowerBehavior.BRAKE
        else
            NextMotor.ZeroPowerBehavior.FLOAT
        flyWheelLeft?.let {
            it.zeroPowerBehavior = zpb
            it.setVelocitySetpoint(0.0.rpm)
        }
        flyWheelRight?.let {
            it.zeroPowerBehavior = zpb
            it.setVelocitySetpoint(0.0.rpm)
        }
    }

    /** Measured LEFT wheel speed from the encoder, RPM (0.0 when absent). */
    fun measuredLeftRpm(): Double = flyWheelLeft?.encoderVelocity?.into(RotationsPerMinute) ?: 0.0

    /** Measured RIGHT wheel speed from the encoder, RPM (0.0 when absent). */
    fun measuredRightRpm(): Double = flyWheelRight?.encoderVelocity?.into(RotationsPerMinute) ?: 0.0

    /**
     * Average measured speed across the wheels that EXIST, RPM (0.0 with no
     * wheels). Averaging both raw readings counts a missing wheel as 0 and
     * silently halves every RPM/dip threshold on a bench robot.
     */
    fun measuredAverageRpm(): Double {
        val speeds = listOfNotNull(
            flyWheelLeft?.encoderVelocity?.into(RotationsPerMinute),
            flyWheelRight?.encoderVelocity?.into(RotationsPerMinute),
        )
        return if (speeds.isEmpty()) 0.0 else speeds.average()
    }

    /**
     * True when every EXISTING wheel is within [toleranceRpm] of [targetRpm].
     * This is the "safe to fire" check — launching on slow wheels is a short
     * shot. A zero-wheel flywheel is never at speed.
     */
    fun isAtSpeed(targetRpm: Double, toleranceRpm: Double = FLYWHEEL_SPEED_TOLERANCE_RPM): Boolean {
        if (!anyMotor) return false
        if (flyWheelLeft != null && kotlin.math.abs(measuredLeftRpm() - targetRpm) > toleranceRpm) return false
        if (flyWheelRight != null && kotlin.math.abs(measuredRightRpm() - targetRpm) > toleranceRpm) return false
        return true
    }
}




/**
 * ============ BALL LOADER — the indexer between hopper and flywheel ============
 *
 * A goBILDA dual-mode servo in POSITION mode (config name [LoaderServoName])
 * meters exactly ONE ball per stroke. Firing is now deterministic loader
 * strokes — the intake does NOT run while shooting. All timings/positions live
 * in globals.kt ([LOADER_HOME_POSITION] … [LOADER_RETRACT_MS]); measure them
 * on the real robot.
 *
 * The stroke/retract cycle is time-based (standard PWM servos report no
 * position feedback to the hub), so [busy] is the source of truth for "is it
 * still moving" and [phase] settles automatically once the motion time ends.
 */
enum class LoaderPhase { HOME, STROKING, STROKED, RETRACTING }

class Loader : Mechanism {

    private val servo = NextServo(LoaderServoName)

    private var livePhase = LoaderPhase.HOME
    private var motionEndNanos = 0L

    /**
     * True when no "loader" servo exists in the hardware configuration
     * (e.g. a motors-only bring-up robot). Every call becomes a safe no-op
     * instead of crashing the OpMode on the first hardware-map access.
     */
    @Volatile
    var missing = false
        private set

    /** False when the servo isn't configured — call sites can adapt. */
    val available: Boolean get() = !missing

    /** Runs a servo write; on any hardware-map failure marks [missing] once. */
    private fun touch(action: () -> Unit): Boolean {
        if (missing) return false
        return try {
            action()
            true
        } catch (e: Exception) {
            missing = true
            BlackBox.event("loader_missing", "msg" to (e.message ?: "?"))
            false
        }
    }

    /** Completed strokes since [resetCount] — the real "balls fed" counter. */
    var strokeCount = 0
        private set

    /** Current mechanical phase; settled lazily once motion time expires. */
    val phase: LoaderPhase
        get() {
            if (System.nanoTime() >= motionEndNanos) {
                livePhase = when (livePhase) {
                    LoaderPhase.STROKING -> LoaderPhase.STROKED
                    LoaderPhase.RETRACTING -> LoaderPhase.HOME
                    else -> livePhase
                }
            }
            return livePhase
        }

    /** True while the servo is still traveling. */
    val busy: Boolean
        get() = System.nanoTime() < motionEndNanos

    // NOTE: no auto-home in init — early-constructed singletons can be    // constructed before the hardware map exists. OpModes call home() once
    // inside runOpMode(), where hardware is guaranteed ready.

    /** Commands the servo to the home (gate closed) position. */
    fun home() {
        if (!touch { servo.position = LOADER_HOME_POSITION }) return
        livePhase = LoaderPhase.RETRACTING
        motionEndNanos = System.nanoTime() + (LOADER_RETRACT_MS * 1e6).toLong()
    }

    /**
     * Meters ONE ball into the flywheel (home -> stroke end).
     * Only works from HOME when not busy; returns false otherwise (safe to
     * call every loop tick — that's how "hold X to empty the hopper" works).
     */
    fun stroke(): Boolean {
        if (busy || phase != LoaderPhase.HOME) return false
        if (!touch { servo.position = LOADER_STROKE_END_POSITION }) return false
        livePhase = LoaderPhase.STROKING
        motionEndNanos = System.nanoTime() + (LOADER_STROKE_MS * 1e6).toLong()
        strokeCount++
        BlackBox.event("loader_stroke", "stroke" to strokeCount)
        return true
    }

    /**
     * Returns the servo home so the next stroke can index another ball.
     * (If your mechanism ratchets — indexing only on the forward stroke —
     * the retract is mechanically harmless by design; if not, set the stroke
     * positions so the cycle works for your hardware.)
     */
    fun retract(): Boolean {
        if (busy || phase != LoaderPhase.STROKED) return false
        if (!touch { servo.position = LOADER_HOME_POSITION }) return false
        livePhase = LoaderPhase.RETRACTING
        motionEndNanos = System.nanoTime() + (LOADER_RETRACT_MS * 1e6).toLong()
        return true
    }

    fun resetCount() {
        strokeCount = 0
    }
}

/**
 * ============ SHOT DETECTOR — "did we actually shoot?" ============
 *
 * Confirms each fired ball from the FLYWHEEL ENCODERS: a ball hitting the
 * spinning wheels briefly loads them, so measured RPM dips and recovers.
 * Arm right after a loader stroke completes; a dip below
 * [SHOT_DIP_THRESHOLD_RPM] from the at-fire baseline within
 * [SHOT_DETECT_WINDOW_MS] = CONFIRMED, else MISSED.
 *
 * Every verdict is written to the Black Box — those (distance, RPM, dip)
 * records are exactly what tools/shooter_refit.py learns the shooter curve
 * from after real matches.
 */
class ShotDetector(private val flywheel: FlyWheel) {

    enum class State { IDLE, ARMED, CONFIRMED, MISSED }

    var state = State.IDLE
        private set
    var lastDipRpm = 0.0
        private set

    private var baselineRpm = 0.0
    private var minRpm = Double.MAX_VALUE
    private var windowEndNanos = 0L
    private var confirmedCount = 0

    /** Average measured speed of the wheels that exist, RPM; 0.0 if hardware is gone. */
    fun measuredRpm(): Double = try {
        flywheel.measuredAverageRpm()
    } catch (_: Exception) {
        0.0
    }

    /** Arms dip detection. Call immediately after a loader stroke finishes. */
    fun arm() {
        baselineRpm = measuredRpm()
        minRpm = Double.MAX_VALUE
        windowEndNanos = System.nanoTime() + (SHOT_DETECT_WINDOW_MS * 1e6).toLong()
        state = State.ARMED
    }

    /** Call every loop tick while ARMED. Transitions only via the dip/timeout. */
    fun update(): State {
        if (state != State.ARMED) return state
        val rpm = measuredRpm()
        if (rpm < minRpm) minRpm = rpm

        if (minRpm < baselineRpm - SHOT_DIP_THRESHOLD_RPM) {
            state = State.CONFIRMED
            lastDipRpm = baselineRpm - minRpm
            confirmedCount++
            BlackBox.event(
                "shot_confirmed",
                "dipRpm" to lastDipRpm,
                "baselineRpm" to baselineRpm,
                "minRpm" to minRpm,
            )
        } else if (System.nanoTime() >= windowEndNanos) {
            state = State.MISSED
            BlackBox.event(
                "shot_missed",
                "baselineRpm" to baselineRpm,
                "minRpm" to minRpm,
            )
        }
        return state
    }

    fun totalConfirmed(): Int = confirmedCount

    fun idle() {
        state = State.IDLE
    }
}
