package org.firstinspires.ftc.teamcode

import kotlin.math.PI
import kotlin.math.roundToInt
import kotlin.math.sqrt

/**
 * ============ DRIVETRAIN SPEC — measure once, compute everything ============
 *
 * Fill in the PHYSICAL MEASUREMENTS below (tape measure + motor spec sheet)
 * and every derived constant in the codebase is computed from them:
 *   * encoder ticks per wheel rev          ([driveTicksPerWheelRev])
 *   * cm per encoder tick                  ([driveCmPerTick])
 *   * Road Runner's inPerTick/trackWidthTicks defaults
 *   * cm -> wheel-radian / tick conversions ([cmToWheelTicks], [cmToWheelRad])
 *   * in-place turn geometry                ([turnRadiusCm])
 *   * dead-wheel pod geometry               ([deadWheelsIn])
 *
 * NO HAND MATH, NO UNIT MIXUPS. The encoder/position/velocity chain is
 * consistent by construction: NextMotor is created with
 * `anglePerCount = 2π / driveTicksPerWheelRev`, so encoderPosition /
 * encoderVelocity / position setpoints are all WHEEL radians — the conversions
 * below therefore NEVER multiply by the gear ratio (the old cmToRad did, and
 * every position move would have overshot by 51.46x).
 */
object DrivetrainSpec {

    // ================== PHYSICAL MEASUREMENTS — FILL THESE IN ==================

    // -- Drive wheels (goBILDA 5203-series Yellow Jackets, mecanum) --
    /** MEASURE: wheel diameter at the ground, cm (loaded, not nominal). */
    const val DRIVE_WHEEL_DIAMETER_CM = 10.4
    /** SPEC SHEET: motor-shaft encoder counts per revolution. */
    const val DRIVE_MOTOR_CPR = 28.0
    /** SPEC SHEET: gearbox reduction (motor revs per wheel rev). 312 RPM = 19.2:1. */
    const val DRIVE_GEAR_RATIO = 19.2
    /** MEASURE: left/right wheel contact-point center distance, cm. */
    const val TRACK_WIDTH_CM = 42.0
    /** MEASURE: front/rear axle center distance, cm. */
    const val WHEELBASE_CM = 32.0
    /** MEASURE: how much a loaded mecanum slides sideways vs driving (1.0 = ideal). */
    const val MECANUM_LATERAL_SLIP = 1.0

    // -- Dead-wheel odometry pods (bolt on: par0 / par1 / perp motors) --
    /** MEASURE: odometry pod wheel diameter, cm (SWYFT Linear Odometry Module: 38mm). */
    const val DEAD_WHEEL_DIAMETER_CM = 3.8
    /** SPEC SHEET: odometry pod counts per rev (SWYFT: 4096 PPR quadrature). */
    const val DEAD_WHEEL_TICKS_PER_REV = 4096.0
    /** MEASURE: par0 position from the drive center, cm (+x forward, +y left). */
    const val PAR0_X_CM = -16.0
    const val PAR0_Y_CM = 8.5
    /** MEASURE: par1 position from the drive center, cm. */
    const val PAR1_X_CM = -16.0
    const val PAR1_Y_CM = -8.5
    /** MEASURE: perp position from the drive center, cm. */
    const val PERP_X_CM = -1.0
    const val PERP_Y_CM = 21.0
    /**
     * True when the pod's encoder COUNTS UP while the robot is pushed
     * FORWARD (par/perp) / LEFT (perp). Determined by which way you plugged
     * the pod in — the "Drivetrain Setup & Verify" OpMode steps you through
     * it; set the flag to match, never rewire the hub.
     */
    const val PAR0_REVERSED = false
    const val PAR1_REVERSED = false
    const val PERP_REVERSED = false

    // ================== DERIVED — computed, do not edit ==================

    /** Encoder counts per WHEEL revolution (motor CPR through the gearbox). */
    val driveTicksPerWheelRev: Double = DRIVE_MOTOR_CPR * DRIVE_GEAR_RATIO

    /** Centimeters of travel per encoder tick (one full tick of wheel rotation). */
    val driveCmPerTick: Double = PI * DRIVE_WHEEL_DIAMETER_CM / driveTicksPerWheelRev

    /**
     * Free speed of ONE drive motor in wheel RPM — the maximum the motor can
     * produce (the gearbox ratio is already folded into the wheel RPM).
     *
     * goBILDA 5203-2402-0019 = 312 RPM. This is the number that makes the
     * Road Runner feedforward derivable instead of tuned (see [rrKV]).
     */
    const val DRIVE_MOTOR_MAX_RPM = 312.0

    /**
     * Free speed of the drive wheels with the motor at full power, in
     * inches/sec of wheel travel.
     *
     *     v_free = RPM * pi * diameter / 60  (in cm/sec), then / 2.54
     */
    val driveFreeSpeedInPerSec: Double =
        DRIVE_MOTOR_MAX_RPM * PI * DRIVE_WHEEL_DIAMETER_CM / 60.0 / 2.54

    /**
     * Road Runner's velocity feedforward, DERIVED from the hardware so the
     * robot can drive without a tuning session.
     *
     * Road Runner 1.0 computes motor power as
     *
     *     power = (kS * sign(v) + kV * v_ticks + kA * a_ticks) / batteryVolts
     *
     * where `v_ticks` is wheel velocity in TICKS per second. Note the two
     * things that are easy to get wrong here:
     *
     *   1. `kS` and `kV` are in VOLTS, not in normalized power. Road Runner
     *      divides the result by the measured battery voltage, so `kV` is
     *      "volts of motor output per tick/sec of wheel speed".
     *   2. The code applies `kV / inPerTick` to a velocity in INCHES/sec, so
     *      `kV` must be in "volts per tick/sec" while the effective gain is
     *      in "volts per inch/sec".
     *
     * At full power the motor drives the wheel at [driveFreeSpeedInPerSec].
     * So the feedforward that asks for exactly the battery voltage at free
     * speed satisfies:
     *
     *     kS + (kV / inPerTick) * v_free = batteryVolts
     *  => kV = inPerTick * (batteryVolts - kS) / v_free
     *
     * This is the physically correct starting point: full power => full
     * speed. It is deliberately conservative — the real value is a little
     * HIGHER because a loaded wheel never quite reaches free speed, which
     * shows up as mild under-driving that the feedback gains absorb, rather
     * than the violent over-driving that a too-low kV causes.
     */
    val rrKV: Double = driveCmPerTick / 2.54 * (RR_BATTERY_VOLTS - RR_KS) / driveFreeSpeedInPerSec

    /**
     * Static-friction feedforward, in VOLTS: the motor output needed just to
     * break the robot loose. 0.6 V is about 5% of a 12.6 V battery — enough
     * to start moving, not enough to lurch. (For reference, a published
     * Road Runner 1.0 tuning session on a similar mecanum landed at 0.78 V.)
     */
    const val RR_KS = 0.6

    /**
     * Nominal battery voltage used for the derivation. Road Runner divides by
     * the MEASURED voltage at runtime, so this only sets the reference point.
     */
    const val RR_BATTERY_VOLTS = 12.6

    /**
     * Acceleration feedforward, in volts per tick/sec^2. Small by design: the
     * position controller handles the bulk of the acceleration, and an
     * over-large kA makes the robot lurch at the start of every trajectory.
     */
    const val RR_KA = 0.0

    /** Road Runner's inPerTick: encoder ticks -> inches of travel. */
    val rrInPerTick: Double = driveCmPerTick / 2.54

    /** Road Runner's lateralInPerTick (mecanum strafing is not free). */
    val rrLateralInPerTick: Double = rrInPerTick * MECANUM_LATERAL_SLIP

    /** Road Runner's trackWidthTicks: track width expressed in encoder ticks. */
    val rrTrackWidthTicks: Double = (TRACK_WIDTH_CM / 2.54) / rrInPerTick

    /** Odometry pod inches of travel per encoder tick. */
    val deadWheelInPerTick: Double = PI * DEAD_WHEEL_DIAMETER_CM / DEAD_WHEEL_TICKS_PER_REV / 2.54

    /** cm of robot travel -> wheel radians (NextFTC position units). */
    fun cmToWheelRad(cm: Double): Double = 2.0 * cm / DRIVE_WHEEL_DIAMETER_CM

    /** cm of robot travel -> encoder ticks. */
    fun cmToWheelTicks(cm: Double): Int =
        (cm / (PI * DRIVE_WHEEL_DIAMETER_CM) * driveTicksPerWheelRev).roundToInt()

    /** Radius of the circle each wheel traces in an in-place spin, cm. */
    fun turnRadiusCm(): Double = sqrt(WHEELBASE_CM * WHEELBASE_CM + TRACK_WIDTH_CM * TRACK_WIDTH_CM) / 2.0

    /** Degrees of in-place spin -> wheel-radian rotation for each side. */
    fun turnToWheelRad(degrees: Double): Double = cmToWheelRad(turnRadiusCm() * degrees * PI / 180.0)

    /**
     * Dead-wheel geometry in Road Runner inches: (x, y, angleRad) per pod,
     * par wheels roll with driving (angle 0), perp with strafing (PI/2).
     */
    fun deadWheelsIn(): List<DeadWheelMath.WheelSpec> = listOf(
        DeadWheelMath.WheelSpec(PAR0_X_CM / 2.54, PAR0_Y_CM / 2.54, 0.0),
        DeadWheelMath.WheelSpec(PAR1_X_CM / 2.54, PAR1_Y_CM / 2.54, 0.0),
        DeadWheelMath.WheelSpec(PERP_X_CM / 2.54, PERP_Y_CM / 2.54, PI / 2),
    )

    fun deadWheelReversed(): List<Boolean> = listOf(PAR0_REVERSED, PAR1_REVERSED, PERP_REVERSED)

    /** Everything the setup bench prints, as telemetry-ready pairs. */
    fun summary(): List<Pair<String, String>> = listOf(
        "ticks / wheel rev" to "%.1f (spec: CPR %.0f x ratio %.2f)".format(
            driveTicksPerWheelRev, DRIVE_MOTOR_CPR, DRIVE_GEAR_RATIO,
        ),
        "travel / tick" to "%.5f cm (%.5f in)".format(driveCmPerTick, rrInPerTick),
        "RR inPerTick" to "%.6f".format(rrInPerTick),
        "RR lateralInPerTick" to "%.6f".format(rrLateralInPerTick),
        "RR trackWidthTicks" to "%.1f (track %.1f cm)".format(rrTrackWidthTicks, TRACK_WIDTH_CM),
        "turn radius" to "%.1f cm (wheelbase %.1f, track %.1f)".format(
            turnRadiusCm(), WHEELBASE_CM, TRACK_WIDTH_CM,
        ),
        "dead wheel in/tick" to "%.6f (d=%.2fcm, %.0f ticks/rev)".format(
            deadWheelInPerTick, DEAD_WHEEL_DIAMETER_CM, DEAD_WHEEL_TICKS_PER_REV,
        ),
        "par0" to "(%.1f, %.1f) cm%s".format(PAR0_X_CM, PAR0_Y_CM, if (PAR0_REVERSED) " [REV]" else ""),
        "par1" to "(%.1f, %.1f) cm%s".format(PAR1_X_CM, PAR1_Y_CM, if (PAR1_REVERSED) " [REV]" else ""),
        "perp" to "(%.1f, %.1f) cm%s".format(PERP_X_CM, PERP_Y_CM, if (PERP_REVERSED) " [REV]" else ""),
        "50 cm straight-line" to "= %d ticks (tape-measure this!)".format(cmToWheelTicks(50.0)),
        "90 deg spin" to "= %.1f wheel rad/side".format(turnToWheelRad(90.0)),
    )
}
