package org.firstinspires.ftc.teamcode

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Tests for the MATCH POLICY — the decisions that decide whether the robot
 * banks its pollen and still parks.
 *
 * These rules used to live inline in the OpMode, where no test could reach
 * them. Every one of them has a consequence in a real match: failing to park
 * costs a penalty, starting a score run you cannot finish throws away the
 * balls you are carrying, and parking while the hopper is full wastes the
 * whole autonomous period.
 */
class PhasePolicyTest {

    private val policy = PhasePolicy(
        maxBallsHeld = 4,
        collectTimeoutSec = 9.0,
        flowerPullTimeoutSec = 6.0,
        polishSec = 4.0,
        parkSec = 2.0,
        collectPerBallSec = 2.5,
        endgameMarginSec = 8.0,
    )

    /** A mid-match robot: empty hopper, plenty of time, flowers around. */
    private fun midMatch(
        ballsHeld: Int = 0,
        alignedForFire: Boolean = false,
        polishDone: Boolean = false,
        collectDoneThisCycle: Boolean = false,
        flowerDoneThisCycle: Boolean = false,
        remainingSec: Double = 30.0,
        reserveSec: Double = 8.0,
        flowerAvailable: Boolean = true,
        flowerClusterAvailable: Boolean = true,
        hiveXcm: Double? = 90.0,
        hiveYcm: Double? = 60.0,
        robotXcm: Double = 0.0,
        robotYcm: Double = 0.0,
    ) = PhaseState(
        ballsHeld = ballsHeld,
        alignedForFire = alignedForFire,
        polishDone = polishDone,
        collectDoneThisCycle = collectDoneThisCycle,
        flowerDoneThisCycle = flowerDoneThisCycle,
        robotXcm = robotXcm,
        robotYcm = robotYcm,
        hiveXcm = hiveXcm,
        hiveYcm = hiveYcm,
        flowerClusterAvailable = flowerClusterAvailable,
        flowerAvailable = flowerAvailable,
        remainingSec = remainingSec,
        reserveSec = reserveSec,
    )

    private fun List<PhaseTask>.task(phase: AutonomyPhase): PhaseTask =
        first { it.phase == phase }

    // ================= priorities =================

    @Test
    fun `the phase priorities encode the match strategy`() {
        val c = policy.candidates(midMatch())
        assertEquals(80, c.task(AutonomyPhase.FIRE).priority)
        assertEquals(70, c.task(AutonomyPhase.ALIGN).priority)
        assertEquals(65, c.task(AutonomyPhase.PARK).priority)
        // The script's next phase gets the elevated priority; the others
        // sit at the baseline.
        assertEquals(60, c.task(AutonomyPhase.COLLECT).priority)
        assertEquals(50, c.task(AutonomyPhase.FLOWER).priority)
        assertEquals(50, c.task(AutonomyPhase.POLISH).priority)
    }

    @Test
    fun `parking outranks working but never outranks scoring`() {
        val c = policy.candidates(midMatch())
        assertTrue(
            "a ball we cannot score is worth nothing: park beats collect",
            c.task(AutonomyPhase.PARK).priority > c.task(AutonomyPhase.COLLECT).priority,
        )
        assertTrue(
            "banking before parking is always better",
            c.task(AutonomyPhase.ALIGN).priority > c.task(AutonomyPhase.PARK).priority,
        )
        assertTrue(
            c.task(AutonomyPhase.FIRE).priority > c.task(AutonomyPhase.PARK).priority,
        )
    }

    @Test
    fun `parking is mandatory and never preemptible`() {
        val park = policy.candidates(midMatch()).task(AutonomyPhase.PARK)
        assertTrue("parking must never be dropped", park.mandatory)
        assertFalse("parking must never be interrupted", park.canPreempt)
    }

    @Test
    fun `the polish pass is never interrupted once started`() {
        assertFalse(policy.candidates(midMatch()).task(AutonomyPhase.POLISH).canPreempt)
    }

    // ================= collection =================

    @Test
    fun `collecting is executable while there is capacity`() {
        assertTrue(policy.candidates(midMatch(ballsHeld = 0)).task(AutonomyPhase.COLLECT).executable)
    }

    @Test
    fun `collecting stops when the hopper is full`() {
        assertFalse(policy.candidates(midMatch(ballsHeld = 4)).task(AutonomyPhase.COLLECT).executable)
    }

    @Test
    fun `a full hopper makes scoring mandatory`() {
        val c = policy.candidates(midMatch(ballsHeld = 4, remainingSec = 30.0))
        assertTrue("a full hopper must trigger a score run", c.task(AutonomyPhase.ALIGN).executable)
        assertTrue(policy.mustScore(midMatch(ballsHeld = 4, remainingSec = 30.0)))
    }

    @Test
    fun `collecting does not run when the hopper is full`() {
        assertFalse(policy.candidates(midMatch(ballsHeld = 4)).task(AutonomyPhase.COLLECT).executable)
    }

    // ================= flowers =================

    @Test
    fun `a flower pull needs an available flower and spare capacity`() {
        assertTrue(policy.candidates(midMatch(ballsHeld = 0)).task(AutonomyPhase.FLOWER).executable)
        assertFalse(
            "no flower means no flower phase",
            policy.candidates(midMatch(flowerAvailable = false)).task(AutonomyPhase.FLOWER).executable,
        )
        assertFalse(
            "no capacity means no flower phase",
            policy.candidates(midMatch(ballsHeld = 4)).task(AutonomyPhase.FLOWER).executable,
        )
    }

    @Test
    fun `a single loose flower counts even without a cluster`() {
        val c = policy.candidates(
            midMatch(flowerAvailable = true, flowerClusterAvailable = false),
        )
        assertTrue("a loose flower is still nectar", c.task(AutonomyPhase.FLOWER).executable)
    }

    // ================= firing and alignment =================

    @Test
    fun `firing needs balls and an alignment`() {
        assertFalse(policy.candidates(midMatch(ballsHeld = 2)).task(AutonomyPhase.FIRE).executable)
        assertTrue(
            policy.candidates(midMatch(ballsHeld = 2, alignedForFire = true))
                .task(AutonomyPhase.FIRE).executable,
        )
        assertFalse(
            "aligned but empty means nothing to fire",
            policy.candidates(midMatch(ballsHeld = 0, alignedForFire = true))
                .task(AutonomyPhase.FIRE).executable,
        )
    }

    @Test
    fun `the score run is already paid for by the reserve`() {
        // FIRE costs nothing on top of the reserve: the reserve IS the money
        // for spin-up, feeding and parking.
        assertEquals(0.0, policy.candidates(midMatch()).task(AutonomyPhase.FIRE).estimatedCostSec, 0.0)
    }

    @Test
    fun `aligning only happens when the endgame calls for it`() {
        // plenty of time, nothing urgent: no reason to line up yet
        assertFalse(policy.candidates(midMatch(ballsHeld = 1, remainingSec = 40.0))
            .task(AutonomyPhase.ALIGN).executable)
        // time has run down to where the endgame no longer fits later
        assertTrue(policy.candidates(midMatch(ballsHeld = 1, remainingSec = 16.0))
            .task(AutonomyPhase.ALIGN).executable)
    }

    @Test
    fun `the robot banks what it holds instead of collecting more`() {
        // one ball short of a full hopper costs 2.5 s to gather, plus the
        // 8 s reserve: below 15.5 s left, gathering more is pointless.
        val c = policy.candidates(
            midMatch(ballsHeld = 3, remainingSec = 14.0, reserveSec = 8.0),
        )
        assertTrue(
            "gathering more pollen we cannot score is pointless",
            c.task(AutonomyPhase.ALIGN).executable,
        )
        assertTrue(policy.mustScore(midMatch(ballsHeld = 3, remainingSec = 14.0, reserveSec = 8.0)))
    }

    @Test
    fun `the robot keeps collecting while the cycle still fits`() {
        // three balls short of a full hopper (7.5 s) plus the 8 s reserve
        // still fits in 20 s
        val c = policy.candidates(
            midMatch(ballsHeld = 1, remainingSec = 20.0, reserveSec = 8.0),
        )
        assertFalse(
            "there is still time to fill the hopper and score",
            c.task(AutonomyPhase.ALIGN).executable,
        )
    }

    // ================= parking: the safety net =================

    @Test
    fun `the robot parks when the running phase can no longer finish`() {
        val collect = policy.candidates(midMatch(remainingSec = 40.0)).task(AutonomyPhase.COLLECT)
        // the collect phase needs 9 s, the reserve is 8 s: 17 s of room needed
        assertTrue(
            "a phase that cannot finish must yield to parking",
            policy.candidates(
                midMatch(remainingSec = 12.0, reserveSec = 8.0),
                running = collect,
                runningElapsedSec = 0.0,
            ).task(AutonomyPhase.PARK).executable,
        )
    }

    @Test
    fun `the robot parks rather than stand still when nothing else fits`() {
        // hopper full (no collecting, no flowers) and the endgame reserve
        // no longer fits in what is left
        val c = policy.candidates(
            midMatch(ballsHeld = 4, remainingSec = 9.0, reserveSec = 12.0),
        )
        assertTrue("standing still at the end is a penalty", c.task(AutonomyPhase.PARK).executable)
    }

    @Test
    fun `the robot does not park while there is work to do`() {
        assertFalse(
            policy.candidates(midMatch(remainingSec = 30.0, reserveSec = 8.0))
                .task(AutonomyPhase.PARK).executable,
        )
    }

    @Test
    fun `parking stays executable even with plenty of time when nothing else can run`() {
        // full hopper, aligned: firing IS possible, so no park yet
        val firing = policy.candidates(
            midMatch(ballsHeld = 4, alignedForFire = true, remainingSec = 40.0),
        )
        assertTrue(firing.task(AutonomyPhase.FIRE).executable)
        assertFalse(firing.task(AutonomyPhase.PARK).executable)

        // full hopper, NOT aligned, and not even the endgame reserve fits:
        // park instead of wandering around looking for a hive
        val stranded = policy.candidates(
            midMatch(ballsHeld = 4, alignedForFire = false, remainingSec = 10.0, reserveSec = 12.0),
        )
        assertTrue(stranded.task(AutonomyPhase.PARK).executable)
    }

    @Test
    fun `a running phase that still fits does not force an early park`() {
        val collect = policy.candidates(midMatch(remainingSec = 40.0)).task(AutonomyPhase.COLLECT)
        val c = policy.candidates(
            midMatch(remainingSec = 30.0, reserveSec = 8.0),
            running = collect,
            runningElapsedSec = 1.0,
        )
        assertFalse("there is still time to collect", c.task(AutonomyPhase.PARK).executable)
    }

    @Test
    fun `the robot never parks at the start of the match`() {
        // The reserve is large early on; a policy that let the reserve
        // trigger a park would park the robot with 20 s left and score
        // nothing.
        val c = policy.candidates(
            midMatch(ballsHeld = 0, remainingSec = 30.0, reserveSec = 11.0),
        )
        assertFalse("the match starts with collecting, not parking", c.task(AutonomyPhase.PARK).executable)
    }

    // ================= the match script =================

    @Test
    fun `the script collects first`() {
        val c = policy.candidates(midMatch(collectDoneThisCycle = false))
        assertTrue(
            "collection comes first in the script",
            c.task(AutonomyPhase.COLLECT).priority > c.task(AutonomyPhase.FLOWER).priority,
        )
    }

    @Test
    fun `the script pulls a flower after collecting`() {
        val c = policy.candidates(midMatch(collectDoneThisCycle = true, flowerDoneThisCycle = false))
        assertTrue(
            "the flower comes next in the script",
            c.task(AutonomyPhase.FLOWER).priority > c.task(AutonomyPhase.COLLECT).priority,
        )
    }

    @Test
    fun `the script polishes after the flower`() {
        val c = policy.candidates(
            midMatch(collectDoneThisCycle = true, flowerDoneThisCycle = true, polishDone = false),
        )
        assertTrue(
            "the cleanup comes after the flower",
            c.task(AutonomyPhase.POLISH).priority > c.task(AutonomyPhase.COLLECT).priority,
        )
    }

    @Test
    fun `the script skips the flower when no flower is available`() {
        val c = policy.candidates(
            midMatch(collectDoneThisCycle = true, flowerAvailable = false, polishDone = false),
        )
        assertTrue(
            "no flower means no waiting for one",
            c.task(AutonomyPhase.POLISH).priority > c.task(AutonomyPhase.COLLECT).priority,
        )
    }

    @Test
    fun `the script restarts collecting once the cycle is done`() {
        val c = policy.candidates(
            midMatch(collectDoneThisCycle = true, flowerDoneThisCycle = true, polishDone = true),
        )
        assertTrue(
            "a finished cycle starts a new one",
            c.task(AutonomyPhase.COLLECT).priority > c.task(AutonomyPhase.FLOWER).priority,
        )
    }

    // ================= mustScore =================

    @Test
    fun `mustScore agrees with the alignment trigger`() {
        assertTrue(policy.mustScore(midMatch(ballsHeld = 4, remainingSec = 40.0)))
        assertFalse(policy.mustScore(midMatch(ballsHeld = 0, remainingSec = 40.0)))
        assertTrue(policy.mustScore(midMatch(ballsHeld = 1, remainingSec = 16.0)))
        assertFalse(policy.mustScore(midMatch(ballsHeld = 1, remainingSec = 40.0)))
    }
}
