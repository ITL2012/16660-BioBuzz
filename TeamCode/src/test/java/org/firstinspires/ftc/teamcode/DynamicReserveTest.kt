package org.firstinspires.ftc.teamcode

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * JVM tests for the dynamic endgame reserves.
 *
 * A flat "reserve 7 seconds" is wrong in both directions: on the far side
 * of the field the robot needs a long run home, and while parked in a
 * corner it needs almost nothing. The calculator derives the reserve from
 * live state, and these tests pin the three things it scales with:
 * the run home, the number of balls held, and the distance to the wall.
 */
class DynamicReserveTest {

    private fun calc(
        robot: Pair<Double, Double>? = 0.0 to 0.0,
        hive: Pair<Double, Double>? = 100.0 to 0.0,
        balls: Int = 1,
        wallDist: Double? = null,
        aligned: Boolean = false,
    ) = DynamicReserveCalculator(
        robotPositionCm = { robot },
        hivePositionCm = { hive },
        ballsHeld = { balls },
        distanceToWallCm = { wallDist },
        isAligned = { aligned },
    )

    @Test
    fun `the fire reserve grows with the number of balls held`() {
        val one = calc(balls = 1).fireReserveSec()
        val four = calc(balls = 4).fireReserveSec()
        assertTrue("four balls ($four) must cost more than one ($one)", four > one)
    }

    @Test
    fun `the fire reserve grows with the run home`() {
        val near = calc(robot = 90.0 to 0.0).fireReserveSec()
        val far = calc(robot = -100.0 to 0.0).fireReserveSec()
        assertTrue("the long run home ($far) must cost more than the short one ($near)", far > near)
    }

    @Test
    fun `the fire reserve never exceeds its clamp`() {
        val absurd = calc(robot = -1000.0 to -1000.0, balls = 4).fireReserveSec()
        assertTrue("the reserve was $absurd", absurd <= RESERVE_FIRE_MAX_SEC + 1e-9)
    }

    @Test
    fun `an unknown hive is treated pessimistically`() {
        val known = calc(hive = 100.0 to 0.0).fireReserveSec()
        val unknown = calc(hive = null).fireReserveSec()
        assertFalse(calc(hive = null).hasHiveFix())
        assertTrue("unknown hive ($unknown) must reserve at least as much as $known", unknown >= known)
    }

    @Test
    fun `the park reserve is larger in the open field than against a wall`() {
        val againstWall = calc(wallDist = 10.0).parkReserveSec()
        val outInTheOpen = calc(wallDist = 200.0).parkReserveSec()
        assertTrue(
            "the open field ($outInTheOpen) must reserve more than the wall ($againstWall)",
            outInTheOpen > againstWall,
        )
    }

    @Test
    fun `the park reserve never exceeds its clamp`() {
        val absurd = calc(wallDist = 10000.0).parkReserveSec()
        assertTrue("the reserve was $absurd", absurd <= RESERVE_PARK_MAX_SEC + 1e-9)
    }

    @Test
    fun `the total reserve is the whole endgame`() {
        val c = calc(balls = 2, wallDist = 200.0)
        assertEquals(
            c.alignReserveSec() + c.fireReserveSec() + c.parkReserveSec(),
            c.totalReserveSec(),
            1e-9,
        )
    }

    @Test
    fun `the reserve covers lining up on the hive`() {
        val c = calc(balls = 2)
        assertEquals(ALIGN_ESTIMATE_SEC, c.alignReserveSec(), 1e-9)
        assertTrue(
            "the endgame must include the alignment",
            c.totalReserveSec() > c.fireReserveSec() + c.parkReserveSec(),
        )
    }

    @Test
    fun `the alignment drops out of the reserve once the robot is lined up`() {
        // Once aligned, reserving the alignment again would make the robot
        // believe it cannot afford to fire.
        val notAligned = calc(balls = 2)
        val aligned = calc(balls = 2, aligned = true)
        assertEquals(0.0, aligned.alignReserveSec(), 1e-9)
        assertTrue(
            "aligning must free up reserve",
            aligned.totalReserveSec() < notAligned.totalReserveSec(),
        )
    }

    @Test
    fun `the clock clamps a phase deadline by the dynamic reserve`() {
        var simNanos = 0L
        var reserve = 2.0
        val clock = PhaseClock(
            periodSec = 30.0,
            nowNanos = { simNanos },
            reserveCalculator = { reserve },
        )
        simNanos += (25.0 * 1e9).toLong() // 5s left
        val deadline = clock.deadlineNanos(9000.0)
        // the phase wants 9s, but only 5 - 2 (reserve) = 3s are available
        assertEquals(simNanos + 3.0e9, deadline.toDouble(), 1e6)
    }

    @Test
    fun `a growing reserve shrinks the time a phase may take`() {
        var simNanos = 0L
        var reserve = 1.0
        val clock = PhaseClock(
            periodSec = 30.0,
            nowNanos = { simNanos },
            reserveCalculator = { reserve },
        )
        simNanos += (20.0 * 1e9).toLong() // 10s left
        val withSmallReserve = clock.deadlineNanos(9000.0)
        reserve = 5.0
        val withBigReserve = clock.deadlineNanos(9000.0)
        assertTrue(
            "the bigger reserve must shorten the budget",
            withBigReserve - simNanos < withSmallReserve - simNanos,
        )
    }

    @Test
    fun `no budget at all when the reserve eats everything`() {
        var simNanos = 0L
        val clock = PhaseClock(
            periodSec = 30.0,
            nowNanos = { simNanos },
            reserveCalculator = { 5.0 },
        )
        simNanos += (28.0 * 1e9).toLong() // 2s left, reserve 5s
        assertEquals(simNanos, clock.deadlineNanos(9000.0))
    }
}
