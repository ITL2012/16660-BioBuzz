package org.firstinspires.ftc.teamcode

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test
import kotlin.math.PI

/**
 * Pure-math regression tests for the 3-dead-wheel kinematics. The classic
 * FTC layout is used: two parallel wheels at (0, +6) and (0, -6) inches
 * (angle 0) and one perpendicular wheel at (-6, 0) (angle PI/2).
 */
class DeadWheelMathTest {

    private val par0 = DeadWheelMath.WheelSpec(0.0, 6.0, 0.0)
    private val par1 = DeadWheelMath.WheelSpec(0.0, -6.0, 0.0)
    private val perp = DeadWheelMath.WheelSpec(-6.0, 0.0, PI / 2)
    private val wheels = listOf(par0, par1, perp)

    @Test
    fun `pure forward motion recovers dx`() {
        // both parallel wheels roll 10in, perpendicular wheel still
        val (dx, dy) = DeadWheelMath.positionDelta(wheels, listOf(10.0, 10.0, 0.0), 0.0)
        assertEquals(10.0, dx, 1e-9)
        assertEquals(0.0, dy, 1e-9)
    }

    @Test
    fun `pure strafe-left motion recovers dy`() {
        // perpendicular wheel rolls +5in for 5in of leftward body motion
        val (dx, dy) = DeadWheelMath.positionDelta(wheels, listOf(0.0, 0.0, 5.0), 0.0)
        assertEquals(0.0, dx, 1e-9)
        assertEquals(5.0, dy, 1e-9)
    }

    @Test
    fun `pure rotation produces zero translation`() {
        // in-place rotation CCW by 0.1 rad: each wheel reads its own arc,
        // which the model must subtract back to zero translation
        val d = 0.1
        val arc0 = (par0.x * kotlin.math.sin(par0.angleRad) - par0.y * kotlin.math.cos(par0.angleRad)) * d
        val arc1 = (par1.x * kotlin.math.sin(par1.angleRad) - par1.y * kotlin.math.cos(par1.angleRad)) * d
        val arcP = (perp.x * kotlin.math.sin(perp.angleRad) - perp.y * kotlin.math.cos(perp.angleRad)) * d
        val (dx, dy) = DeadWheelMath.positionDelta(wheels, listOf(arc0, arc1, arcP), d)
        assertEquals(0.0, dx, 1e-9)
        assertEquals(0.0, dy, 1e-9)
    }

    @Test
    fun `forward plus rotation separates cleanly`() {
        val d = 0.1
        val f = 10.0
        // wheel i measures forward + its own rotation arc
        fun arc(w: DeadWheelMath.WheelSpec) =
            (w.x * kotlin.math.sin(w.angleRad) - w.y * kotlin.math.cos(w.angleRad)) * d
        val (dx, dy) = DeadWheelMath.positionDelta(
            wheels,
            listOf(f + arc(par0), f + arc(par1), arc(perp)),
            d,
        )
        assertEquals(f, dx, 1e-9)
        assertEquals(0.0, dy, 1e-9)
    }

    @Test
    fun `velocity solve - pure forward`() {
        val (vx, vy, omega) = DeadWheelMath.velocity(wheels, listOf(20.0, 20.0, 0.0))
        assertEquals(20.0, vx, 1e-9)
        assertEquals(0.0, vy, 1e-9)
        assertEquals(0.0, omega, 1e-9)
    }

    @Test
    fun `velocity solve - strafe and rotation`() {
        // strafing left 5 in/s AND rotating CCW 1 rad/s:
        // par0 reads vy*(0) + ... = -y0*omega = -6 ; par1 = +6 ; perp = vy + x*omega = 5 - 6 = -1
        val (vx, vy, omega) = DeadWheelMath.velocity(wheels, listOf(-6.0, 6.0, -1.0))
        assertEquals(0.0, vx, 1e-9)
        assertEquals(5.0, vy, 1e-9)
        assertEquals(1.0, omega, 1e-9)
    }

    @Test
    fun `precomputed kinematics match the direct solve`() {
        val kinematics = DeadWheelMath.Kinematics(wheels)
        val positions = doubleArrayOf(10.2, 9.8, 3.1)
        val velocities = doubleArrayOf(-6.0, 6.0, -1.0)
        val headingDelta = 0.08

        val expectedPosition = DeadWheelMath.positionDelta(wheels, positions.toList(), headingDelta)
        val actualPosition = kinematics.positionDelta(positions, headingDelta)
        assertEquals(expectedPosition.first, actualPosition.first, 1e-9)
        assertEquals(expectedPosition.second, actualPosition.second, 1e-9)

        val expectedVelocity = DeadWheelMath.velocity(wheels, velocities.toList())
        val actualVelocity = kinematics.velocity(velocities)
        assertEquals(expectedVelocity.first, actualVelocity.first, 1e-9)
        assertEquals(expectedVelocity.second, actualVelocity.second, 1e-9)
        assertEquals(expectedVelocity.third, actualVelocity.third, 1e-9)
    }

    @Test
    fun `wrapDeg wraps into the -180 to +180 range`() {
        assertEquals(-170.0, DeadWheelMath.wrapDeg(190.0), 1e-9)
        assertEquals(170.0, DeadWheelMath.wrapDeg(-190.0), 1e-9)
        assertEquals(-180.0, DeadWheelMath.wrapDeg(180.0), 1e-9)
        assertEquals(-180.0, DeadWheelMath.wrapDeg(-180.0), 1e-9)
        assertEquals(0.0, DeadWheelMath.wrapDeg(360.0), 1e-9)
    }

    @Test
    fun `inchesPerTick and sign handling`() {
        assertEquals(Math.PI * 1.37795 / 2000.0, DeadWheelMath.inchesPerTick(2000.0, 1.37795), 1e-12)
        assertEquals(-5.0, DeadWheelMath.signed(5.0, true), 1e-12)
        assertEquals(5.0, DeadWheelMath.signed(5.0, false), 1e-12)
    }
}
