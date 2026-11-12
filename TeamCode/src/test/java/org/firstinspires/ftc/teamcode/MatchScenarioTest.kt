package org.firstinspires.ftc.teamcode

import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.min

/**
 * ============ FULL-MATCH SCENARIO TESTS ============
 *
 * The unit tests pin individual rules. This one drives the whole strategy:
 * the [PhasePolicy] and the [PhaseScheduler] together, against a simulated
 * 30-second autonomous period, and checks that the robot actually plays a
 * sensible match.
 *
 * The robot is a crude model — it picks up balls when it collects, scores
 * when it fires — but the DECISIONS are the real ones: the real policy, the
 * real scheduler, the real clock with the real reserve arithmetic.
 *
 * The three things that matter on the field:
 *
 *  1. the robot parks, and parks in time (an unparked robot is penalised),
 *  2. the robot scores the pollen it collects (collecting without scoring
 *     is worth nothing),
 *  3. the robot does not park with 20 seconds still on the clock (that
 *     means the strategy gave up early).
 */
class MatchScenarioTest {

    /** The simulated robot's state machine. */
    private class SimRobot(val periodSec: Double = 30.0) {
        var nowNanos = 0L
        val clock = PhaseClock(periodSec = periodSec, nowNanos = { nowNanos })
        val policy = PhasePolicy()
        val scheduler = PhaseScheduler(clock)

        var ballsHeld = 0
        var aligned = false
        var polishDone = false
        var collectDone = false
        var flowerDone = false

        /** Balls the robot has scored so far. */
        var scored = 0

        /** The phase sequence the robot actually ran. */
        val phases = mutableListOf<String>()

        /** Wall-clock seconds the robot spent in each phase. */
        val durations = mutableListOf<Pair<String, Double>>()

        /**
         * The endgame reserve, modelled the way the real clock does it:
         * the fire reserve grows with the number of balls we have to feed.
         */
        fun reserveSec(): Double =
            (if (aligned) 0.0 else ALIGN_ESTIMATE_SEC) +
                1.0 + 3.0 + 0.8 * ballsHeld + 2.0

        fun state() = PhaseState(
            ballsHeld = ballsHeld,
            alignedForFire = aligned,
            polishDone = polishDone,
            collectDoneThisCycle = collectDone,
            flowerDoneThisCycle = flowerDone,
            robotXcm = 0.0,
            robotYcm = 0.0,
            hiveXcm = 60.0,
            hiveYcm = 40.0,
            flowerClusterAvailable = true,
            flowerAvailable = true,
            remainingSec = clock.remainingSec(),
            reserveSec = reserveSec(),
        )

        /** How long the phase really takes on the robot. */
        private fun actualCost(phase: AutonomyPhase): Double = when (phase) {
            AutonomyPhase.COLLECT -> min(9.0, (4.0 - ballsHeld).coerceAtLeast(1.0) * 3.0)
            AutonomyPhase.FLOWER -> 6.0
            AutonomyPhase.POLISH -> 4.0
            AutonomyPhase.ALIGN -> 4.0
            AutonomyPhase.FIRE -> 3.0 + 1.2 * ballsHeld
            AutonomyPhase.PARK -> 2.0
        }

        /** Runs the match. Returns the seconds left when the robot stopped. */
        fun play(): Double {
            var guard = 0
            while (clock.remainingSec() > 0.0 && guard < 50) {
                guard++
                val task = scheduler.next(
                    policy.candidates(state(), scheduler.current, scheduler.currentElapsedSec()),
                ) ?: break

                phases += task.phase.name
                val cost = actualCost(task.phase)
                durations += task.phase.name to cost

                when (task.phase) {
                    AutonomyPhase.COLLECT -> {
                        ballsHeld = min(4, ballsHeld + (cost / 2.5).toInt().coerceAtLeast(1))
                        collectDone = true
                    }
                    AutonomyPhase.FLOWER -> flowerDone = true
                    AutonomyPhase.POLISH -> polishDone = true
                    AutonomyPhase.ALIGN -> aligned = true
                    AutonomyPhase.FIRE -> {
                        scored += ballsHeld
                        ballsHeld = 0
                        aligned = false
                        collectDone = false
                        flowerDone = false
                    }
                    AutonomyPhase.PARK -> {
                        // the park is the last thing the robot does
                        nowNanos += (cost * 1e9).toLong()
                        return clock.remainingSec()
                    }
                }

                nowNanos += (cost * 1e9).toLong()
                scheduler.complete()
            }
            return clock.remainingSec()
        }
    }

    @Test
    fun `the robot parks before the period ends`() {
        val robot = SimRobot()
        val left = robot.play()

        assertTrue(
            "the robot must park, but it ran ${robot.phases}",
            robot.phases.contains("PARK"),
        )
        assertTrue(
            "the robot must park with time left, but it had ${left}s",
            left >= 0.0,
        )
        assertTrue(
            "the robot parked too late: ${left}s left",
            left > 0.5,
        )
    }

    @Test
    fun `the robot scores the pollen it collects`() {
        val robot = SimRobot()
        robot.play()

        assertTrue(
            "the robot collected but never scored: ${robot.phases}",
            robot.scored > 0,
        )
    }

    @Test
    fun `the robot does not give up at the start of the match`() {
        val robot = SimRobot()
        robot.play()

        val worked = robot.durations
            .filter { it.first != "PARK" }
            .sumOf { it.second }
        assertTrue(
            "the robot only worked ${worked}s of a 30s period",
            worked >= 10.0,
        )
    }

    @Test
    fun `the robot keeps working until the endgame forces it to stop`() {
        val robot = SimRobot()
        robot.play()

        // The park must come near the end, not in the middle: the robot
        // should still be collecting while there is time to score.
        val parkIndex = robot.phases.indexOf("PARK")
        assertTrue("the park must be the last phase: ${robot.phases}", parkIndex == robot.phases.size - 1)
        assertTrue(
            "the robot parked after only ${robot.phases.size - 1} phases: ${robot.phases}",
            robot.phases.size - 1 >= 3,
        )
    }

    @Test
    fun `a full hopper sends the robot to score immediately`() {
        val robot = SimRobot()
        // Start with a full hopper: the robot must score, not collect.
        robot.ballsHeld = 4
        robot.play()

        assertTrue(
            "a full hopper must be scored, got: ${robot.phases}",
            robot.phases.indexOf("ALIGN") < robot.phases.indexOf("COLLECT").let { if (it < 0) Int.MAX_VALUE else it },
        )
        assertTrue("the full hopper must be scored: ${robot.phases}", robot.scored >= 4)
    }

    @Test
    fun `the robot parks even when it cannot score what it holds`() {
        val robot = SimRobot()
        // 20 seconds already gone, nothing in the hopper: the only sane
        // move is to park.
        robot.nowNanos = (20.0 * 1e9).toLong()
        robot.play()

        assertTrue(
            "the robot must park when the time is gone: ${robot.phases}",
            robot.phases.contains("PARK"),
        )
    }
}
