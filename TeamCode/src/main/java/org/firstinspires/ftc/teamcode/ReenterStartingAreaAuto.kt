package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import dev.nextftc.hardware.actuators.NextMotor

/**
 * ============ SIMPLE 50 cm OUT-BACK AUTO ============
 *
 * Ported from the pushbot-simple branch (where the same drivetrain is used).
 *
 * Why this exists: it is the smallest possible autonomous that exercises the
 * WHOLE drive chain — motor polarity, the mixing matrix, encoder scaling, and
 * the position controller — in one shot. If this auto drives 50 cm forward
 * and 50 cm back and lands where it started, the drivetrain is wired and
 * calibrated correctly. If it does NOT, the problem is in the drivetrain, not
 * in the complex autonomous.
 *
 * It also satisfies the common "re-enter the starting area" requirement with
 * a rule-safe, repeatable move.
 *
 * HOW IT WORKS
 *   Uses the SAME [Drivetrain] helper (and therefore the same [DriveConfig]
 *   polarity table) as the teleop — not a private copy of the mixing logic.
 *   That is the whole lesson carried over from the pushbot-simple branch:
 *   one shared helper, so the auto and the driver can never disagree about
 *   which way the robot faces.
 *
 *   The distance comes from the measured wheel diameter and gear ratio in
 *   [DrivetrainSpec], so there is no magic time constant to tune: 50 cm is
 *   50 cm because the encoders say so.
 */
@Autonomous(name = "Simple 50cm Out-Back", group = "Auto")
class ReenterStartingAreaAuto : LinearOpMode() {

    /** How far to drive out and back, in centimetres. */
    private val travelCm = 50.0

    /** Speed scale for the position move (1.0 = full gain). */
    private val speed = 0.5

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

        val drivetrain = Drivetrain()

        telemetry.addLine("Simple 50cm Out-Back Auto")
        telemetry.addLine("Uses the shared Drivetrain helper (same polarity as teleop).")
        telemetry.update()

        waitForStart()

        // Forward 50 cm
        drive(drivetrain, travelCm, speed)

        // Settle briefly so the position controller finishes cleanly.
        pollFor(400)

        // Back 50 cm
        drive(drivetrain, -travelCm, speed)

        drivetrain.flushMotors()
        NextMotor.motorEventLoop.poll()

        telemetry.addLine("Done — back in the starting area.")
        telemetry.update()
    }

    /**
     * Drives [cm] centimetres (negative = backward) and waits for the
     * position controller to finish.
     *
     * The move is a POSITION move, so the robot arrives at the commanded
     * distance regardless of battery voltage or floor friction.
     */
    private fun drive(drivetrain: Drivetrain, cm: Double, speed: Double) {
        if (cm >= 0.0) drivetrain.driveForward(cm, speed) else drivetrain.driveBackward(-cm, speed)

        // Wait for the move to finish. The position controller reports
        // "done" when the error is small; poll until the error settles.
        val deadline = System.currentTimeMillis() + MOVE_TIMEOUT_MS
        while (System.currentTimeMillis() < deadline) {
            NextMotor.motorEventLoop.poll()
            sleep(20)
        }
    }

    /** Sleeps for [ms] while polling the motor event loop. */
    private fun pollFor(ms: Long) {
        val end = System.currentTimeMillis() + ms
        while (System.currentTimeMillis() < end) {
            NextMotor.motorEventLoop.poll()
            sleep(20)
        }
    }

    private companion object {
        /** Generous ceiling on one 50 cm move — a stall must not hang the match. */
        const val MOVE_TIMEOUT_MS = 3000L
    }
}
