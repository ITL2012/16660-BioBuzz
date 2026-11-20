package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.acmerobotics.roadrunner.Pose2d
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import java.util.Locale
import kotlin.math.abs

/**
 * ============ WHEEL DIRECTION TEST — the 60-second truth test ============
 *
 * Ported from the pushbot-simple branch (where the same drivetrain is used).
 * This is the FIRST thing to run on a freshly wired robot: it tells you
 * which motor is which and whether its polarity is correct, before any
 * tuning, calibration, or tuning-by-feel is needed.
 *
 * CONTROLS
 *   Hold A  = frontleft only
 *   Hold B  = frontright only
 *   Hold X  = backleft only
 *   Hold Y  = backright only
 *   D-pad UP = all four wheels forward at once
 *
 * HOW TO READ THE RESULT
 *   * A DIFFERENT wheel moves than the button you pressed
 *       → the hub config name for that motor is wrong (or the motor is
 *         plugged into a different port). Fix the Robot Configuration, or
 *         change the *_NAME constants in globals.kt. Do NOT fight it with
 *         direction flags.
 *   * D-pad UP makes the robot STRAFE or SPIN instead of driving forward
 *       → one or more motor polarities are wrong. Flip the matching flag(s)
 *         in driveconfig.kt (FL_REVERSED / FR_REVERSED / BL_REVERSED /
 *         BR_REVERSED) and redeploy. ONE flag, not the hub wiring.
 *   * The encoder number for that wheel stays FROZEN while it spins
 *       → the encoder wire is loose or the encoder is dead. The motor will
 *         still spin in raw-power mode here, but closed-loop control on that
 *         wheel will not work until the encoder is fixed.
 *
 * NOTE ON "FORWARD" — on a mecanum the two sides are mirrored, so FL/BL spin
 * one rotational way and FR/BR spin the OTHER way. That is CORRECT. Do not
 * try to make every wheel spin the same way; the only thing that must hold
 * is that D-pad UP drives the robot FORWARD.
 *
 * This OpMode uses RAW POWER on purpose: it must work even when an encoder
 * is dead, which is exactly when you most need to know the wiring.
 */
@TeleOp(name = "Wheel Direction Test", group = "Test")
class WheelDirectionTest : LinearOpMode() {

    override fun runOpMode() {
        val hw = HardwareProbe.probe(hardwareMap)
        if (!hw.driveMotors) {
            while (!isStopRequested) {
                telemetry.addLine("!! DRIVE MOTORS NOT DETECTED !!")
                telemetry.addLine("Check the Driver Station configuration.")
                telemetry.update()
                sleep(50)
            }
            return
        }

        // Read the motors directly (not through NextFTC) so this test works
        // with a dead encoder and does not depend on any control loop.
        val fl = hardwareMap.get(DcMotorEx::class.java, DrivetrainFrontLeft)
        val fr = hardwareMap.get(DcMotorEx::class.java, DrivetrainFrontRight)
        val bl = hardwareMap.get(DcMotorEx::class.java, DrivetrainBackLeft)
        val br = hardwareMap.get(DcMotorEx::class.java, DrivetrainBackRight)

        // Apply the polarity table from driveconfig.kt — the SAME table the
        // robot uses in every other OpMode. If this test is wrong, the robot
        // is wrong; they can never disagree.
        fl.direction = if (DriveConfig.FL_REVERSED) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
        fr.direction = if (DriveConfig.FR_REVERSED) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
        bl.direction = if (DriveConfig.BL_REVERSED) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
        br.direction = if (DriveConfig.BR_REVERSED) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD

        // Always raw power for this test, regardless of any encoder state.
        listOf(fl, fr, bl, br).forEach {
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        val testPower = 0.4

        telemetry.addLine("WHEEL DIRECTION TEST")
        telemetry.addLine("A=frontleft  B=frontright  X=backleft  Y=backright")
        telemetry.addLine("D-pad UP = all four forward")
        telemetry.addLine("Expected: the named wheel rolls FORWARD")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            fl.power = if (gamepad1.a) testPower else 0.0
            fr.power = if (gamepad1.b) testPower else 0.0
            bl.power = if (gamepad1.x) testPower else 0.0
            br.power = if (gamepad1.y) testPower else 0.0
            if (gamepad1.dpad_up) {
                fl.power = testPower
                fr.power = testPower
                bl.power = testPower
                br.power = testPower
            }

            telemetry.addData("frontleft", "%s  enc=%d".format(DriveConfig.FL_REVERSED, fl.currentPosition))
            telemetry.addData("frontright", "%s  enc=%d".format(DriveConfig.FR_REVERSED, fr.currentPosition))
            telemetry.addData("backleft", "%s  enc=%d".format(DriveConfig.BL_REVERSED, bl.currentPosition))
            telemetry.addData("backright", "%s  enc=%d".format(DriveConfig.BR_REVERSED, br.currentPosition))
            telemetry.addLine("TRUE/FALSE = REVERSED flag. Enc frozen while spinning = dead encoder.")
            telemetry.update()
        }

        fl.power = 0.0
        fr.power = 0.0
        bl.power = 0.0
        br.power = 0.0
    }
}

/**
 * ============ FLYWHEEL DIRECTION TEST — which way does each wheel spin? ============
 *
 * Verifies the flywheel spin direction (see FLYWHEEL_LEFT_REVERSED /
 * FLYWHEEL_RIGHT_REVERSED in globals.kt, and README Chapter 10.4).
 *
 * CONTROLS
 *   Hold A = spin LEFT wheel only
 *   Hold B = spin RIGHT wheel only
 *   Hold X = spin BOTH (this is what the launcher actually does)
 *   Hold Y = stop
 *
 * HOW TO READ THE RESULT
 *   * Both wheels must COUNTER-ROTATE. If they spin the same way, the ball
 *     cannot be squeezed and will squirt out sideways — flip one constant.
 *   * If the ball launches BACKWARD out of the launcher, flip BOTH constants.
 *   * Measured RPM should be POSITIVE for both wheels (the encoder sign
 *     follows the spin direction). A negative reading for a wheel that is
 *     spinning normally means that wheel's direction flag is flipped.
 */
@TeleOp(name = "Flywheel Direction Test", group = "Test")
class FlywheelDirectionTest : LinearOpMode() {

    override fun runOpMode() {
        val hw = HardwareProbe.probe(hardwareMap)
        if (!hw.anyFlywheel) {
            while (!isStopRequested) {
                telemetry.addLine("!! NO FLYWHEEL MOTORS DETECTED !!")
                telemetry.addData("Missing", hw.missingList.joinToString(", "))
                telemetry.update()
                sleep(50)
            }
            return
        }

        val flywheel = FlyWheel(hasLeft = hw.flywheelLeft, hasRight = hw.flywheelRight)
        val testRpm = 2000.0

        telemetry.addLine("FLYWHEEL DIRECTION TEST")
        telemetry.addLine("A=left only  B=right only  X=both  Y=stop")
        telemetry.addLine("The two wheels MUST counter-rotate.")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            dev.nextftc.hardware.actuators.NextMotor.motorEventLoop.poll()

            val target = if (gamepad1.y) 0.0 else testRpm

            // Each mode drives exactly one wheel (or both) so a reversed
            // wheel is unambiguous — the measured RPM sign shows it directly.
            when {
                gamepad1.a -> flywheel.spinOneWheelToRpm(left = true, rpm = target)
                gamepad1.b -> flywheel.spinOneWheelToRpm(left = false, rpm = target)
                gamepad1.x -> flywheel.spinToRpm(target)
                else -> flywheel.stop()
            }

            telemetry.addData(
                "Commanded",
                String.format(Locale.US, "%.0f RPM", target),
            )
            telemetry.addData(
                "LEFT measured",
                String.format(Locale.US, "%.0f RPM  (reversed=%s)", flywheel.measuredLeftRpm(), FLYWHEEL_LEFT_REVERSED),
            )
            telemetry.addData(
                "RIGHT measured",
                String.format(Locale.US, "%.0f RPM  (reversed=%s)", flywheel.measuredRightRpm(), FLYWHEEL_RIGHT_REVERSED),
            )
            telemetry.update()
        }

        flywheel.stop(breakMotor = true)
        dev.nextftc.hardware.actuators.NextMotor.motorEventLoop.poll()
    }
}

/**
 * ============ DRIVE DIRECTION CHECK — is the robot driving the right way? ============
 *
 * A companion to the Wheel Direction Test: it rolls the robot forward and
 * backward a fixed distance and reports what the ODOMETRY thinks happened.
 * This catches the failure that a per-wheel test cannot: a robot that spins
 * in place, strafes when told to go forward, or whose pose drifts sideways.
 *
 * CONTROLS
 *   A (tap) = drive forward 50 cm
 *   B (tap) = drive backward 50 cm
 *   X (hold) = live pose + wheel encoder readout
 *
 * EXPECTED
 *   After A: pose x increases by ~19.7 in (50 cm), heading unchanged.
 *   After B: pose x returns to where it started.
 *   If the pose moves sideways instead, the encoder directions do not match
 *   the motor directions — see the note in roadrunner/MecanumDrive.kt.
 */
@TeleOp(name = "Drive Direction Check", group = "Test")
class DriveDirectionCheck : LinearOpMode() {

    override fun runOpMode() {
        val hw = HardwareProbe.probe(hardwareMap)
        if (!hw.driveMotors) {
            while (!isStopRequested) {
                telemetry.addLine("!! DRIVE MOTORS NOT DETECTED !!")
                telemetry.update()
                sleep(50)
            }
            return
        }

        val drivetrain = Drivetrain()
        val locator = try {
            MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
        } catch (_: Exception) {
            null
        }

        telemetry.addLine("DRIVE DIRECTION CHECK")
        telemetry.addLine("A = forward 50cm | B = backward 50cm | X(hold) = live pose")
        telemetry.update()
        waitForStart()

        var lastA = false
        var lastB = false

        while (opModeIsActive()) {
            dev.nextftc.hardware.actuators.NextMotor.motorEventLoop.poll()
            locator?.updatePoseEstimate()

            if (gamepad1.a && !lastA) {
                drivetrain.driveForward(50.0, 0.25)
                sleepPolling(3000)
                drivetrain.flushMotors()
                dev.nextftc.hardware.actuators.NextMotor.motorEventLoop.poll()
            }
            if (gamepad1.b && !lastB) {
                drivetrain.driveBackward(50.0, 0.25)
                sleepPolling(3000)
                drivetrain.flushMotors()
                dev.nextftc.hardware.actuators.NextMotor.motorEventLoop.poll()
            }
            lastA = gamepad1.a
            lastB = gamepad1.b

            if (gamepad1.x && locator != null) {
                val p = locator.pose
                telemetry.addData(
                    "Pose (in)",
                    "x=%.2f  y=%.2f  hdg=%.1f deg".format(
                        p.position.x, p.position.y, Math.toDegrees(p.heading.toDouble()),
                    ),
                )
                telemetry.addData("Expect after A", "x=+19.69  y=0  hdg=0")
                telemetry.addData("Localizer", if (locator.usingDeadWheels) "dead wheels" else "drive motors")
            } else {
                for (pair in DriveConfig.summary()) telemetry.addData(pair.first, pair.second)
            }
            telemetry.update()
        }

        drivetrain.flushMotors()
        dev.nextftc.hardware.actuators.NextMotor.motorEventLoop.poll()
    }

    /** Sleeps while polling the NextFTC motor event loop so position moves actually run. */
    private fun sleepPolling(ms: Long) {
        val end = System.currentTimeMillis() + ms
        while (System.currentTimeMillis() < end) {
            dev.nextftc.hardware.actuators.NextMotor.motorEventLoop.poll()
            sleep(20)
        }
    }
}
