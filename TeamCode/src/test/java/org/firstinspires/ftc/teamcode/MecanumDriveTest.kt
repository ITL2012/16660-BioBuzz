package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.Time
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive

/**
 * Pure-JVM tests for drive / localizer math. No Android / hardware required:
 *   * MecanumDrive.teleopCommandToVelocity maps normalized TeleOp sticks to Road
 *   * Runner physical units (inches per sec, radians per sec) for push detection.
 *   * The degraded (no-IMU) localizer computes angular velocity from the DualNum
 *   * derivative (measured wheel speed), not the value.
 */
class MecanumDriveTest {

    private val k = MecanumKinematics(1.0, 1.0, 1.0)

    @Test
    fun `teleop full forward maps to +x inches per sec`() {
        val v = MecanumDrive.teleopCommandToVelocity(1.0, 0.0, 0.0)
        assertEquals(DrivetrainSpec.driveFreeSpeedInPerSec, v.component1().x, 1e-9)
        assertEquals(0.0, v.component1().y, 1e-9)
        assertEquals(0.0, v.component2(), 1e-9)
    }

    @Test
    fun `teleop plus strafe right maps to -y because RR +y is left`() {
        val v = MecanumDrive.teleopCommandToVelocity(0.0, 1.0, 0.0)
        assertEquals(0.0, v.component1().x, 1e-9)
        assertEquals(-DrivetrainSpec.driveFreeSpeedInPerSec, v.component1().y, 1e-9)
        assertEquals(0.0, v.component2(), 1e-9)
    }

    @Test
    fun `teleop plus turn clockwise maps to negative angular because RR + is CCW`() {
        val v = MecanumDrive.teleopCommandToVelocity(0.0, 0.0, 1.0)
        assertEquals(0.0, v.component1().x, 1e-9)
        assertEquals(0.0, v.component1().y, 1e-9)
        val turnRadiusIn = DrivetrainSpec.turnRadiusCm() / 2.54
        val expected = -DrivetrainSpec.driveFreeSpeedInPerSec / turnRadiusIn
        assertEquals(expected, v.component2(), 1e-9)
    }

    @Test
    fun `teleop command scales linearly with stick`() {
        val vFull = MecanumDrive.teleopCommandToVelocity(1.0, 0.0, 0.0)
        val vHalf = MecanumDrive.teleopCommandToVelocity(0.5, 0.0, 0.0)
        assertEquals(vFull.component1().x / 2.0, vHalf.component1().x, 1e-9)
    }

    @Test
    fun `zero command is zero velocity`() {
        val v = MecanumDrive.teleopCommandToVelocity(0.0, 0.0, 0.0)
        assertEquals(0.0, v.component1().x, 1e-9)
        assertEquals(0.0, v.component1().y, 1e-9)
        assertEquals(0.0, v.component2(), 1e-9)
    }

    @Test
    fun `mecanum omega comes from the DualNum derivative measured wheel speed`() {
        // Broken form: value set, derivative zero => angular velocity must be zero.
        val noDeriv = MecanumKinematics.WheelIncrements(
            DualNum<Time>(doubleArrayOf(1.0, 0.0)),
            DualNum<Time>(doubleArrayOf(1.0, 0.0)),
            DualNum<Time>(doubleArrayOf(1.0, 0.0)),
            DualNum<Time>(doubleArrayOf(1.0, 0.0)),
        )
        assertEquals(0.0, k.forward(noDeriv).velocity().value().component2(), 1e-9)

        // Correct form: value zero, derivative carries measured wheel speed.
        // Left wheels forward, right wheels back = in-place rotation => nonzero omega.
        val turn = MecanumKinematics.WheelIncrements(
            DualNum<Time>(doubleArrayOf(0.0, 1.0)), // leftFront forward
            DualNum<Time>(doubleArrayOf(0.0, 1.0)), // leftBack forward
            DualNum<Time>(doubleArrayOf(0.0, -1.0)), // rightBack back
            DualNum<Time>(doubleArrayOf(0.0, -1.0)), // rightFront back
        )
        val omega = k.forward(turn).velocity().value().component2()
        assertTrue("in-place rotation should report nonzero omega, got $omega", omega != 0.0)
    }

    @Test
    fun `all wheels at equal speed reports zero omega`() {
        val straight = MecanumKinematics.WheelIncrements(
            DualNum<Time>(doubleArrayOf(0.0, 2.0)),
            DualNum<Time>(doubleArrayOf(0.0, 2.0)),
            DualNum<Time>(doubleArrayOf(0.0, 2.0)),
            DualNum<Time>(doubleArrayOf(0.0, 2.0)),
        )
        assertEquals(0.0, k.forward(straight).velocity().value().component2(), 1e-9)
    }
}
