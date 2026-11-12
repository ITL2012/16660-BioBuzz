package org.firstinspires.ftc.teamcode

/**
 * ============ PHASE SCHEDULER — priority-driven autonomous phase machine ============
 *
 * The old autonomous ran a fixed script: collect -> flower -> polish -> align
 * -> fire -> park, then repeat. That works when every phase has the same
 * urgency, but it cannot answer "the robot is full of pollen and the clock is
 * running out — stop looking for balls and go score NOW". The scheduler
 * replaces the fixed script with a priority queue:
 *
 *  * every phase publishes a [PhaseTask] with a priority, an estimated cost,
 *    and whether it is executable right now,
 *  * the scheduler always runs the highest-priority executable task,
 *  * a task may PREEMPT the running one when its priority beats it by
 *    [SCHEDULER_PREEMPT_PRIORITY_MARGIN] and the running phase still has
 *    meaningful work left ([SCHEDULER_PREEMPT_MIN_REMAINING_SEC]),
 *  * a phase that has already been running is never preempted by a phase
 *    whose remaining time is shorter than the newcomer needs — that would
 *    guarantee the newcomer never completes.
 *
 * The scheduler is pure logic (no hardware, no Android) so the whole policy
 * can be unit-tested on the JVM: which phase runs, whether a switch is legal,
 * and that the endgame always converges on PARK.
 */

/** The autonomous phases the scheduler can arbitrate between. */
enum class AutonomyPhase {
    COLLECT,   // hunt balls with the intake
    FLOWER,    // pull nectar out of a flower
    POLISH,    // one-time vision-guidance cleanup
    ALIGN,     // line up on the hive
    FIRE,      // shoot the held balls
    PARK,      // final back-off (must always happen)
}

/**
 * One candidate phase, as seen by the scheduler.
 *
 * @param phase           which phase this is
 * @param priority        higher wins; only comparable within the same match
 * @param executable      false when the phase cannot run right now (no target,
 *                        no capacity, already done this match, ...)
 * @param estimatedCostSec how long the phase is expected to take
 * @param canPreempt      false for phases that must run to completion
 *                        (POLISH and PARK are not interruptible)
 * @param done            set when the phase is finished for this match
 * @param mandatory       true for the safety-net phase (PARK): it runs even
 *                        when it does not fit in the remaining time, because
 *                        parking late is worse than parking badly.
 */
data class PhaseTask(
    val phase: AutonomyPhase,
    val priority: Int,
    val executable: Boolean,
    val estimatedCostSec: Double,
    val canPreempt: Boolean = true,
    /** Set when the phase is already finished for this match (never re-run). */
    val done: Boolean = false,
    /** Runs even when it does not fit the remaining time (the park). */
    val mandatory: Boolean = false,
) {
    /** A task the scheduler will never start. */
    val blocked: Boolean get() = done || !executable
}

/**
 * Decides which autonomous phase runs next, and whether the running phase
 * should be abandoned for a higher-priority one.
 */
class PhaseScheduler(
    private val clock: PhaseClock,
) {
    /** The task currently running (null before the first [next]). */
    var current: PhaseTask? = null
        private set

    /** When the current task started (nanos on the clock's timebase). */
    var currentStartedNanos: Long = 0L
        private set

    /** How many times a running phase has been preempted. */
    var preemptions: Int = 0
        private set

    /** How many times the scheduler has switched to a different phase. */
    var switches: Int = 0
        private set

    /** Seconds the current phase has been running. */
    fun currentElapsedSec(): Double {
        val started = currentStartedNanos
        if (started == 0L) return 0.0
        return (clock.nowNanos() - started) / 1e9
    }

    /**
     * Picks the task to run now.
     *
     * Returns the running task when nothing should preempt it, the winning
     * candidate when the current phase should yield, or null when no task is
     * executable and affordable.
     */
    fun next(candidates: List<PhaseTask>): PhaseTask? {
        val viable = candidates.filter { !it.blocked && affordable(it) }
        if (viable.isEmpty()) {
            // Nothing affordable: keep running what we have (it may finish
            // and free time), otherwise stand still.
            return current
        }

        val best = viable.maxByOrNull { it.priority } ?: return current

        val running = current
        if (running == null) {
            start(best)
            return best
        }

        if (running.phase == best.phase) {
            // Same phase, maybe a fresher estimate of its cost — keep going.
            if (best.estimatedCostSec != running.estimatedCostSec) {
                current = best
            }
            return current
        }

        if (shouldPreempt(running, best)) {
            preemptions++
            start(best)
            return best
        }
        return running
    }

    /**
     * Whether [candidate] may interrupt [running].
     *
     * Rules, in order:
     *  1. the running phase must be interruptible,
     *  2. the candidate must win by more than the priority margin,
     *  3. the running phase must still have meaningful work left,
     *  4. the candidate must fit in the time the running phase would waste.
     */
    fun shouldPreempt(running: PhaseTask, candidate: PhaseTask): Boolean {
        if (!running.canPreempt) return false
        if (candidate.priority <= running.priority + SCHEDULER_PREEMPT_PRIORITY_MARGIN) return false
        val remainingInRunning = running.estimatedCostSec - currentElapsedSec()
        if (remainingInRunning < SCHEDULER_PREEMPT_MIN_REMAINING_SEC) return false
        // Switching costs the time the running phase would have used anyway;
        // require the newcomer to fit in what is left of the period.
        return affordable(candidate)
    }

    /**
     * The highest-priority candidate that is allowed to preempt [running],
     * or null when nothing should interrupt it.
     *
     * The phase loops call this from inside their own loop so a higher
     * priority phase can take over MID-FLIGHT, rather than waiting for the
     * running phase to finish.
     */
    fun preemptorFor(running: PhaseTask, candidates: List<PhaseTask>): PhaseTask? {
        val best = candidates.filter { !it.blocked }.maxByOrNull { it.priority } ?: return null
        if (best.phase == running.phase) return null
        return if (shouldPreempt(running, best)) best else null
    }

    /** Marks the current task finished (it will not be re-entered). */
    fun complete() {
        current = null
        currentStartedNanos = 0L
    }

    /** True when [phase] is the phase currently running. */
    fun isRunning(phase: AutonomyPhase): Boolean = current?.phase == phase

    /** True when the given phase has been started at least once. */
    private val started = mutableSetOf<AutonomyPhase>()

    /** True when the given phase has ever been started. */
    fun hasStarted(phase: AutonomyPhase): Boolean = phase in started

    private fun start(task: PhaseTask) {
        // The FIRST start is not a switch — a switch is a change of phase
        // while the robot is already doing something.
        val previous = current?.phase
        if (previous != null && previous != task.phase) switches++
        current = task
        currentStartedNanos = clock.nowNanos()
        started.add(task.phase)
    }

    /**
     * A task fits when its own cost plus the clock's reserves fit in what
     * is left. Reserves are what the endgame (park + fire) needs.
     *
     * A MANDATORY task always fits — the park has to happen even if the
     * clock says there is no time for it, because an unparked robot gets
     * penalised.
     */
    private fun affordable(task: PhaseTask): Boolean {
        if (task.mandatory) return true
        val need = task.estimatedCostSec + clock.currentReserveSec()
        return clock.fits(need)
    }
}
