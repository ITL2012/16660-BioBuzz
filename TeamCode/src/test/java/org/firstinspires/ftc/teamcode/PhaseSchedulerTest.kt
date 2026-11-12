package org.firstinspires.ftc.teamcode

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * JVM tests for the priority phase scheduler.
 *
 * The scheduler is the piece that decides WHICH autonomous phase owns the
 * robot, so the rules that matter are:
 *
 *  * the highest-priority executable task wins,
 *  * a task that does not fit in the remaining time is never started,
 *  * the park is mandatory — it runs even when it does not fit,
 *  * a running phase is preempted only when the newcomer wins by more than
 *    the priority margin AND the running phase still has work left,
 *  * a non-preemptible phase (polish, park) is never interrupted.
 */
class PhaseSchedulerTest {

    private var simNanos = 0L
    private val sim: () -> Long get() = { simNanos }

    private fun advance(sec: Double) {
        simNanos += (sec * 1e9).toLong()
    }

    private fun task(
        phase: AutonomyPhase,
        priority: Int,
        executable: Boolean = true,
        cost: Double = 5.0,
        canPreempt: Boolean = true,
        mandatory: Boolean = false,
    ) = PhaseTask(
        phase = phase,
        priority = priority,
        executable = executable,
        estimatedCostSec = cost,
        canPreempt = canPreempt,
        done = false,
        mandatory = mandatory,
    )

    @Test
    fun `highest priority executable task wins`() {
        val clock = PhaseClock(periodSec = 30.0, nowNanos = sim)
        val scheduler = PhaseScheduler(clock)
        val chosen = scheduler.next(
            listOf(
                task(AutonomyPhase.COLLECT, 50),
                task(AutonomyPhase.FLOWER, 40),
                task(AutonomyPhase.POLISH, 10),
            ),
        )
        assertNotNull(chosen)
        assertEquals(AutonomyPhase.COLLECT, chosen!!.phase)
    }

    @Test
    fun `a task that is not executable is never chosen`() {
        val clock = PhaseClock(periodSec = 30.0, nowNanos = sim)
        val scheduler = PhaseScheduler(clock)
        val chosen = scheduler.next(
            listOf(
                task(AutonomyPhase.COLLECT, 50, executable = false),
                task(AutonomyPhase.FLOWER, 40),
            ),
        )
        assertEquals(AutonomyPhase.FLOWER, chosen!!.phase)
    }

    @Test
    fun `a task that does not fit in the remaining time is skipped`() {
        val clock = PhaseClock(periodSec = 30.0, nowNanos = sim)
        advance(25.0) // 5s left
        val scheduler = PhaseScheduler(clock)
        // COLLECT wants 20s but only 5s remain -> not affordable
        val chosen = scheduler.next(
            listOf(
                task(AutonomyPhase.COLLECT, 50, cost = 20.0),
                task(AutonomyPhase.FLOWER, 40, cost = 2.0),
            ),
        )
        assertEquals(AutonomyPhase.FLOWER, chosen!!.phase)
    }

    @Test
    fun `the mandatory park runs even when it does not fit`() {
        val clock = PhaseClock(periodSec = 30.0, nowNanos = sim)
        advance(29.5) // 0.5s left — not even close to a park
        val scheduler = PhaseScheduler(clock)
        val chosen = scheduler.next(
            listOf(
                task(AutonomyPhase.COLLECT, 50, cost = 20.0),
                task(AutonomyPhase.PARK, 100, cost = 8.0, mandatory = true),
            ),
        )
        assertNotNull(chosen)
        assertEquals(AutonomyPhase.PARK, chosen!!.phase)
    }

    @Test
    fun `nothing affordable and nothing running means no task at all`() {
        val clock = PhaseClock(periodSec = 30.0, nowNanos = sim)
        advance(29.0) // 1s left
        val scheduler = PhaseScheduler(clock)
        assertNull(scheduler.next(listOf(task(AutonomyPhase.COLLECT, 50, cost = 20.0))))
    }

    @Test
    fun `a running phase keeps running when nothing better is affordable`() {
        val clock = PhaseClock(periodSec = 30.0, nowNanos = sim)
        val scheduler = PhaseScheduler(clock)
        scheduler.next(listOf(task(AutonomyPhase.COLLECT, 50, cost = 9.0)))
        advance(28.0) // 2s left — the collect phase no longer fits
        val chosen = scheduler.next(listOf(task(AutonomyPhase.COLLECT, 50, cost = 9.0)))
        assertNotNull(chosen)
        assertEquals(AutonomyPhase.COLLECT, chosen!!.phase)
    }

    @Test
    fun `a clearly higher priority task preempts the running one`() {
        val clock = PhaseClock(periodSec = 30.0, nowNanos = sim)
        val scheduler = PhaseScheduler(clock)
        scheduler.next(listOf(task(AutonomyPhase.COLLECT, 50, cost = 9.0)))
        advance(1.0)
        val chosen = scheduler.next(
            listOf(
                task(AutonomyPhase.COLLECT, 50, cost = 9.0),
                task(AutonomyPhase.PARK, 100, cost = 8.0, mandatory = true),
            ),
        )
        assertEquals(AutonomyPhase.PARK, chosen!!.phase)
        assertEquals(1, scheduler.preemptions)
        assertEquals(1, scheduler.switches)
    }

    @Test
    fun `a priority inside the margin does not preempt`() {
        val clock = PhaseClock(periodSec = 30.0, nowNanos = sim)
        val scheduler = PhaseScheduler(clock)
        scheduler.next(listOf(task(AutonomyPhase.COLLECT, 50, cost = 9.0)))
        advance(1.0)
        // 55 is only 5 above 50 — inside the 10-point margin
        val chosen = scheduler.next(
            listOf(
                task(AutonomyPhase.COLLECT, 50, cost = 9.0),
                task(AutonomyPhase.FLOWER, 50 + SCHEDULER_PREEMPT_PRIORITY_MARGIN, cost = 2.0),
            ),
        )
        assertEquals(AutonomyPhase.COLLECT, chosen!!.phase)
        assertEquals(0, scheduler.preemptions)
    }

    @Test
    fun `a phase that is nearly finished is not preempted`() {
        val clock = PhaseClock(periodSec = 30.0, nowNanos = sim)
        val scheduler = PhaseScheduler(clock)
        scheduler.next(listOf(task(AutonomyPhase.COLLECT, 50, cost = 1.0)))
        advance(1.0) // the collect phase is done
        val chosen = scheduler.next(
            listOf(
                task(AutonomyPhase.COLLECT, 50, cost = 1.0),
                task(AutonomyPhase.PARK, 100, cost = 8.0, mandatory = true),
            ),
        )
        assertEquals(AutonomyPhase.COLLECT, chosen!!.phase)
        assertEquals(0, scheduler.preemptions)
    }

    @Test
    fun `a non-preemptible phase is never interrupted`() {
        val clock = PhaseClock(periodSec = 30.0, nowNanos = sim)
        val scheduler = PhaseScheduler(clock)
        scheduler.next(listOf(task(AutonomyPhase.POLISH, 10, cost = 9.0, canPreempt = false)))
        advance(1.0)
        val chosen = scheduler.next(
            listOf(
                task(AutonomyPhase.POLISH, 10, cost = 9.0, canPreempt = false),
                task(AutonomyPhase.PARK, 100, cost = 8.0, mandatory = true),
            ),
        )
        assertEquals(AutonomyPhase.POLISH, chosen!!.phase)
        assertEquals(0, scheduler.preemptions)
    }

    @Test
    fun `preemptorFor reports the intruder from inside the running phase`() {
        val clock = PhaseClock(periodSec = 30.0, nowNanos = sim)
        val scheduler = PhaseScheduler(clock)
        val running = scheduler.next(listOf(task(AutonomyPhase.COLLECT, 50, cost = 9.0)))!!
        advance(1.0)
        val preemptor = scheduler.preemptorFor(
            running,
            listOf(
                task(AutonomyPhase.COLLECT, 50, cost = 9.0),
                task(AutonomyPhase.PARK, 100, cost = 8.0, mandatory = true),
            ),
        )
        assertNotNull(preemptor)
        assertEquals(AutonomyPhase.PARK, preemptor!!.phase)
    }

    @Test
    fun `preemptorFor stays quiet when nothing outranks the running phase`() {
        val clock = PhaseClock(periodSec = 30.0, nowNanos = sim)
        val scheduler = PhaseScheduler(clock)
        val running = scheduler.next(listOf(task(AutonomyPhase.COLLECT, 50, cost = 9.0)))!!
        advance(1.0)
        assertNull(
            scheduler.preemptorFor(
                running,
                listOf(
                    task(AutonomyPhase.COLLECT, 50, cost = 9.0),
                    task(AutonomyPhase.FLOWER, 40, cost = 2.0),
                ),
            ),
        )
    }

    @Test
    fun `completing a phase clears it so the next turn picks afresh`() {
        val clock = PhaseClock(periodSec = 30.0, nowNanos = sim)
        val scheduler = PhaseScheduler(clock)
        scheduler.next(listOf(task(AutonomyPhase.COLLECT, 50)))
        assertTrue(scheduler.isRunning(AutonomyPhase.COLLECT))
        assertTrue(scheduler.hasStarted(AutonomyPhase.COLLECT))
        scheduler.complete()
        assertFalse(scheduler.isRunning(AutonomyPhase.COLLECT))
        assertNull(scheduler.current)
        // the phase can start again, but the scheduler still remembers it ran
        assertTrue(scheduler.hasStarted(AutonomyPhase.COLLECT))
    }

    @Test
    fun `a done task is never selected again`() {
        val clock = PhaseClock(periodSec = 30.0, nowNanos = sim)
        val scheduler = PhaseScheduler(clock)
        val chosen = scheduler.next(
            listOf(
                task(AutonomyPhase.POLISH, 10, executable = true).copy(done = true),
                task(AutonomyPhase.COLLECT, 50),
            ),
        )
        assertEquals(AutonomyPhase.COLLECT, chosen!!.phase)
    }
}
