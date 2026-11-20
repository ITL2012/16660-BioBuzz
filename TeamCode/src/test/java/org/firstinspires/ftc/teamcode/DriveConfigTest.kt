package org.firstinspires.ftc.teamcode

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI
import kotlin.math.abs

/**
 * Locks in the behavior that keeps the teleop and the autonomous agreeing
 * about how the drivetrain is wired (the lesson from the pushbot-simple
 * branch, where the two code paths had drifted apart).
 *
 * The reference is JJ's `TestTeleOP` — the scheme that was proven on this
 * actual robot and that the pushbot-simple branch copied verbatim:
 *
 *     directions:  FL FORWARD, BL FORWARD, FR REVERSE, BR REVERSE
 *     mixing:      fl = y+x+rx   bl = y-x+rx   fr = y-x-rx   br = y+x-rx
 *
 * These tests assert we reproduce that reference EXACTLY, plus the
 * invariants that must hold for any correct mixing.
 */
class DriveConfigTest {

    /** JJ's TestTeleOP mixing, in FL, FR, BL, BR order. */
    private fun jjMix(forward: Double, strafe: Double, turn: Double): DoubleArray {
        val denominator = maxOf(1.0, abs(forward) + abs(strafe) + abs(turn))
        return doubleArrayOf(
            (forward + strafe + turn) / denominator, // FL
            (forward - strafe - turn) / denominator, // FR
            (forward - strafe + turn) / denominator, // BL
            (forward + strafe - turn) / denominator, // BR
        )
    }

    @Test
    fun `mixing reproduces the proven JJ scheme exactly`() {
        for (fwd in listOf(-1.0, -0.3, 0.0, 0.7, 1.0)) {
            for (strafe in listOf(-1.0, -0.3, 0.0, 0.7, 1.0)) {
                for (turn in listOf(-1.0, -0.3, 0.0, 0.7, 1.0)) {
                    val mine = DriveConfig.mix(fwd, strafe, turn)
                    val ref = jjMix(fwd, strafe, turn)
                    for (i in mine.indices) {
                        assertEquals(
                            "mismatch at ($fwd,$strafe,$turn) wheel $i",
                            ref[i], mine[i], 1e-9,
                        )
                    }
                }
            }
        }
    }

    @Test
    fun `polarity flags match the proven JJ motor directions`() {
        // JJ: frontLeft FORWARD, backLeft FORWARD, frontRight REVERSE, backRight REVERSE.
        assertEquals(false, DriveConfig.FL_REVERSED)
        assertEquals(false, DriveConfig.BL_REVERSED)
        assertEquals(true, DriveConfig.FR_REVERSED)
        assertEquals(true, DriveConfig.BR_REVERSED)
    }

    @Test
    fun `forward drives every wheel the same way`() {
        // mix() returns FL, FR, BL, BR.
        val w = DriveConfig.mix(1.0, 0.0, 0.0)
        assertTrue("FL must be positive", w[0] > 0.0)
        assertTrue("FR must be positive", w[1] > 0.0)
        assertTrue("BL must be positive", w[2] > 0.0)
        assertTrue("BR must be positive", w[3] > 0.0)
    }

    @Test
    fun `backward drives every wheel the same way`() {
        val w = DriveConfig.mix(-1.0, 0.0, 0.0)
        assertTrue(w[0] < 0.0)
        assertTrue(w[1] < 0.0)
        assertTrue(w[2] < 0.0)
        assertTrue(w[3] < 0.0)
    }

    @Test
    fun `strafe and turn use opposite wheel pairs`() {
        // Strafe right: the two diagonal wheels (FL, BR) get one sign and the
        // other diagonal (FR, BL) the opposite sign.
        val strafe = DriveConfig.mix(0.0, 1.0, 0.0)
        assertTrue("FL/BR together", strafe[0] * strafe[3] > 0.0)
        assertTrue("FR/BL together", strafe[1] * strafe[2] > 0.0)
        assertTrue("diagonals oppose", strafe[0] * strafe[1] < 0.0)

        // Turn right: the left pair (FL, BL) goes one way, the right pair
        // (FR, BR) the other.
        val turn = DriveConfig.mix(0.0, 0.0, 1.0)
        assertTrue("FL/BL together", turn[0] * turn[2] > 0.0)
        assertTrue("FR/BR together", turn[1] * turn[3] > 0.0)
        assertTrue("sides oppose", turn[0] * turn[1] < 0.0)
    }

    @Test
    fun `no wheel is ever commanded beyond full power`() {
        for (fwd in listOf(-1.0, -0.5, 0.0, 0.5, 1.0)) {
            for (strafe in listOf(-1.0, -0.5, 0.0, 0.5, 1.0)) {
                for (turn in listOf(-1.0, -0.5, 0.0, 0.5, 1.0)) {
                    for (v in DriveConfig.mix(fwd, strafe, turn)) {
                        assertTrue(
                            "command $v out of range for ($fwd,$strafe,$turn)",
                            abs(v) <= 1.0 + 1e-9,
                        )
                    }
                }
            }
        }
    }

    @Test
    fun `mixing is linear and zero input gives zero output`() {
        for (v in DriveConfig.mix(0.0, 0.0, 0.0)) {
            assertEquals(0.0, v, 1e-9)
        }
        // Doubling the input doubles the output (same normalization bucket).
        val one = DriveConfig.mix(0.5, 0.0, 0.0)
        val two = DriveConfig.mix(1.0, 0.0, 0.0)
        for (i in one.indices) {
            assertEquals(two[i], one[i] * 2.0, 1e-9)
        }
    }

    @Test
    fun `opposite commands cancel symmetrically`() {
        // Forward + backward must be exactly zero (no drift in the mix).
        val fwd = DriveConfig.mix(1.0, 0.0, 0.0)
        val back = DriveConfig.mix(-1.0, 0.0, 0.0)
        for (i in fwd.indices) {
            assertEquals(0.0, fwd[i] + back[i], 1e-9)
        }
    }
}

/**
 * Verifies the calibration-free tuning defaults: the Road Runner feedforward
 * and the shooter estimate must be derived from the hardware, not guessed.
 */
class CalibrationFreeDefaultsTest {

    @Test
    fun `road runner feedforward asks for full power at free speed`() {
        // Road Runner 1.0 divides the feedforward by the measured battery
        // voltage, so kS/kV are in VOLTS. The derived values must therefore
        // request exactly the battery voltage when the wheel is at the
        // motor's free speed (full power => full speed).
        val freeSpeed = DrivetrainSpec.driveFreeSpeedInPerSec
        assertTrue("free speed should be realistic", freeSpeed in 10.0..120.0)

        val voltsAtFreeSpeed = DrivetrainSpec.RR_KS +
            (DrivetrainSpec.rrKV / DrivetrainSpec.rrInPerTick) * freeSpeed
        assertEquals(
            "feedforward must request the full battery voltage at free speed",
            DrivetrainSpec.RR_BATTERY_VOLTS, voltsAtFreeSpeed, 1e-9,
        )
    }

    @Test
    fun `feedforward is proportional to speed below free speed`() {
        val half = DrivetrainSpec.driveFreeSpeedInPerSec / 2.0
        val volts = DrivetrainSpec.RR_KS +
            (DrivetrainSpec.rrKV / DrivetrainSpec.rrInPerTick) * half
        // Half the free speed needs roughly half the power (plus friction).
        assertTrue("expected about half power, got $volts", volts < 8.0)
        assertTrue("expected more than a third power, got $volts", volts > 4.0)
    }

    @Test
    fun `free speed matches the motor and wheel specs`() {
        val expected = DrivetrainSpec.DRIVE_MOTOR_MAX_RPM * PI * DrivetrainSpec.DRIVE_WHEEL_DIAMETER_CM / 60.0 / 2.54
        assertEquals(expected, DrivetrainSpec.driveFreeSpeedInPerSec, 1e-9)
    }

    @Test
    fun `static friction is a small fraction of full power`() {
        // kS is in VOLTS (Road Runner divides by the battery voltage), so
        // compare it against the battery voltage, not against 1.0.
        assertTrue(
            "kS must not lurch",
            DrivetrainSpec.RR_KS in 0.2..1.5,
        )
        assertTrue(
            "kS must be a small fraction of the battery",
            DrivetrainSpec.RR_KS / DrivetrainSpec.RR_BATTERY_VOLTS < 0.15,
        )
    }

    @Test
    fun `shooter estimate is monotonic in distance`() {
        val near = ShooterModel.estimatedRpmForDistance(80.0)
        val mid = ShooterModel.estimatedRpmForDistance(150.0)
        val far = ShooterModel.estimatedRpmForDistance(250.0)
        assertTrue("closer needs less RPM", near < mid)
        assertTrue("farther needs more RPM", mid < far)
    }

    @Test
    fun `shooter estimate stays inside the motor's speed range`() {
        for (d in listOf(1.0, 50.0, 100.0, 200.0, 400.0, 1000.0)) {
            val rpm = ShooterModel.estimatedRpmForDistance(d)
            assertTrue("rpm $rpm at ${d}cm too low", rpm >= FLYWHEEL_MIN_FIRING_RPM)
            assertTrue("rpm $rpm at ${d}cm too high", rpm <= FLYWHEEL_MOTOR_MAX_RPM)
        }
    }

    @Test
    fun `uncalibrated shooter does not fire flat out at short range`() {
        // The old behavior (full speed at every distance) is the thing this
        // replaces: a 100 cm shot must NOT command the maximum RPM.
        val rpm = ShooterModel.rpmForDistance(100.0)
        assertTrue("short shot should not fire at max RPM", rpm < FLYWHEEL_MOTOR_MAX_RPM)
    }

    @Test
    fun `zero distance does not divide by zero`() {
        val rpm = ShooterModel.rpmForDistance(0.0)
        assertTrue(rpm.isFinite())
        assertTrue(rpm >= FLYWHEEL_MIN_FIRING_RPM)
    }

    @Test
    fun `estimate stays inside its usable range`() {
        // The estimate's ceiling is where it would ask for more RPM than the
        // flywheel can make. Below it, the estimate must be achievable.
        val ceiling = ShooterModel.estimatedMaxRangeCm()
        assertTrue("ceiling should be far above the shooting window", ceiling > 300.0)

        // The realistic shooting window must be well inside that ceiling, so
        // the uncalibrated robot can always estimate a reachable RPM.
        val window = ShooterModel.effectiveMaxRangeCm()
        assertTrue("shooting window must be inside the estimate's reach", window < ceiling)
        assertTrue(
            "the window's RPM must be achievable",
            ShooterModel.estimatedRpmForDistance(window) <= FLYWHEEL_MOTOR_MAX_RPM,
        )
    }

    @Test
    fun `effective range is the realistic window until calibrated`() {
        // Before the curve is measured, the robot must claim only a
        // plausible shooting window (the hardcoded fallback), NOT the
        // physics estimate's theoretical limit.
        val expected = org.firstinspires.ftc.teamcode.HiveScoringConfig.SHOOT_MAX_RANGE_CM
        assertEquals(expected, ShooterModel.effectiveMaxRangeCm(), 1e-9)
    }
}
