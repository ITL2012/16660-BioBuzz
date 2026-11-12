package org.firstinspires.ftc.teamcode

/**
 * ============ MATCH POLICY — what the robot SHOULD be doing right now ============
 *
 * The [PhaseScheduler] decides which phase owns the robot; the POLICY decides
 * what the phases should be asking for. Keeping the two apart matters:
 *
 *  * the scheduler is the referee (who may interrupt whom),
 *  * the policy is the strategy (what is worth doing at all).
 *
 * The policy is pure logic — no hardware, no Android — so the whole match
 * strategy can be unit-tested on the JVM. That is where the dangerous
 * decisions live: the endgame. A robot that fails to park is penalised, and
 * a robot that starts a score run it cannot finish loses the balls it is
 * carrying. Both are decided here, and both are covered by tests.
 *
 * ================= THE BUDGET MODEL =================
 *
 * The clock holds back a RESERVE for the endgame: the drive to the hive, the
 * flywheel spin-up, feeding every ball we hold, and the park back-off
 * ([DynamicReserveCalculator]). Everything else is work the robot may still
 * do. Each phase declares what it costs ON TOP of that reserve:
 *
 *  * FIRE costs 0 — the reserve is exactly the money for scoring and
 *    parking, so the fire is already paid for.
 *  * ALIGN costs the alignment time. The DRIVE to the hive is inside the
 *    reserve, so charging it twice would make the robot refuse to score.
 *  * COLLECT / FLOWER / POLISH cost their own duration.
 *
 * With that split, "can I afford this phase?" means "can I do this work and
 * still pay for the endgame?", which is the only question that matters.
 *
 * ================= THE PRIORITY ORDER =================
 *
 *  * FIRE (80) — cash in the pollen we hold.
 *  * ALIGN (70) — line up on the hive.
 *  * PARK (65) — the safety net. Above the working phases (a ball we cannot
 *    score is worth nothing) but BELOW the scoring phases (banking before
 *    parking is always better).
 *  * 60 — whichever phase the match script says comes next.
 *  * 50 — the remaining working phases.
 *
 * The script order is the original one the team tuned: collect, then pull a
 * flower, then the one-time vision cleanup, then repeat. The policy tracks
 * that progress in [PhaseState] and hands the script's next phase the
 * elevated priority, so the scheduler reproduces the old behaviour while
 * still being able to interrupt it when the endgame calls.
 */
data class PhaseState(
    /** Balls currently in the hopper. */
    val ballsHeld: Int,
    /** True when the robot is lined up on the hive and ready to fire. */
    val alignedForFire: Boolean,
    /** True once the one-time vision-guidance cleanup has run this match. */
    val polishDone: Boolean,
    /** True once the collection phase has run in the current cycle. */
    val collectDoneThisCycle: Boolean = false,
    /** True once the flower phase has run in the current cycle. */
    val flowerDoneThisCycle: Boolean = false,
    /** Robot position in field cm. */
    val robotXcm: Double,
    val robotYcm: Double,
    /** Best known hive position in field cm (null when never seen). */
    val hiveXcm: Double?,
    val hiveYcm: Double?,
    /** True when a reachable, un-depleted flower CLUSTER is available. */
    val flowerClusterAvailable: Boolean,
    /** True when any usable flower is available (cluster or single). */
    val flowerAvailable: Boolean,
    /** Seconds left in the autonomous period. */
    val remainingSec: Double,
    /** Seconds the endgame still needs (park + fire), from the clock. */
    val reserveSec: Double,
)

/**
 * Builds the candidate phase list the scheduler arbitrates over.
 */
class PhasePolicy(
    private val maxBallsHeld: Int = MAX_BALLS_HELD,
    private val collectTimeoutSec: Double = AUTONOMOUS_COLLECT_TIMEOUT_MS / 1e3,
    private val collectPerBallSec: Double = COLLECT_PER_BALL_SEC,
    private val flowerPullTimeoutSec: Double = FLOWER_PULL_TIMEOUT_MS / 1e3,
    private val polishSec: Double = 4.0,
    private val parkSec: Double = PLANNER_PARK_SEC,
    private val endgameMarginSec: Double = PLANNER_ENDGAME_MARGIN_SEC,
) {

    /**
     * The candidate phases, highest priority first.
     *
     * @param state the live state
     * @param running the phase currently running (null before the first one)
     * @param runningElapsedSec how long that phase has been running
     */
    fun candidates(
        state: PhaseState,
        running: PhaseTask? = null,
        runningElapsedSec: Double = 0.0,
    ): List<PhaseTask> {
        val reserve = state.reserveSec
        val remaining = state.remainingSec
        val holdingBalls = state.ballsHeld > 0
        val hasCapacity = state.ballsHeld < maxBallsHeld

        // --- when must the robot bank what it holds? ---
        // The hopper being full always wins. Otherwise the clock decides:
        // the reserve IS the whole endgame (align, drive home, spin up, feed,
        // park), so once the time left no longer covers the reserve plus a
        // little slack, the score run has to start NOW.
        val mustScore = holdingBalls &&
            (state.ballsHeld >= maxBallsHeld || remaining <= reserve + endgameMarginSec)

        // --- can the robot still finish a whole working cycle? ---
        // Gathering more pollen is pointless when there is no time to fill
        // the hopper AND pay for the endgame afterwards. The time to fill
        // the hopper is what the robot still needs, not the whole
        // collection phase: charging all 9 seconds would make the robot
        // bank a half-full hopper with twenty seconds still on the clock.
        val ballsToFill = (maxBallsHeld - state.ballsHeld).coerceAtLeast(0)
        val fillTimeSec = ballsToFill * collectPerBallSec
        val cycleFits = fillTimeSec + reserve <= remaining
        val bankInsteadOfCollecting = holdingBalls && !cycleFits

        // --- the match script: collect, flower, polish, repeat ---
        val scriptDue: AutonomyPhase = when {
            !state.collectDoneThisCycle -> AutonomyPhase.COLLECT
            !state.flowerDoneThisCycle && state.flowerAvailable -> AutonomyPhase.FLOWER
            !state.polishDone -> AutonomyPhase.POLISH
            else -> AutonomyPhase.COLLECT
        }

        val fireTask = PhaseTask(
            phase = AutonomyPhase.FIRE,
            priority = 80,
            executable = holdingBalls && state.alignedForFire,
            // The reserve IS the money for scoring and parking.
            estimatedCostSec = 0.0,
        )
        val alignTask = PhaseTask(
            phase = AutonomyPhase.ALIGN,
            priority = 70,
            executable = holdingBalls && (mustScore || state.alignedForFire || bankInsteadOfCollecting),
            // The alignment is part of the reserve, so it costs nothing on
            // top: the reserve is exactly the money for lining up, driving
            // home, scoring and parking.
            estimatedCostSec = 0.0,
        )
        val collectTask = PhaseTask(
            phase = AutonomyPhase.COLLECT,
            priority = if (scriptDue == AutonomyPhase.COLLECT) 60 else 50,
            executable = hasCapacity,
            estimatedCostSec = collectTimeoutSec,
        )
        val flowerTask = PhaseTask(
            phase = AutonomyPhase.FLOWER,
            priority = if (scriptDue == AutonomyPhase.FLOWER) 60 else 50,
            executable = state.flowerAvailable && hasCapacity,
            estimatedCostSec = flowerPullTimeoutSec,
        )
        val polishTask = PhaseTask(
            phase = AutonomyPhase.POLISH,
            priority = if (scriptDue == AutonomyPhase.POLISH) 60 else 50,
            executable = !state.polishDone,
            estimatedCostSec = polishSec,
            canPreempt = false,
        )

        val productive = listOf(fireTask, alignTask, collectTask, flowerTask, polishTask)

        // ...and if even the phase we are in cannot finish while still
        // leaving the endgame reserve, we park NOW rather than run over.
        val remainingInRunning = running?.let {
            it.estimatedCostSec - runningElapsedSec
        } ?: 0.0
        val runningCannotFinish = running != null && remaining < remainingInRunning + reserve
        // ...and if NOTHING productive is left that fits, the robot must not
        // sit on the field: parking early beats standing there at the end.
        val nothingElseFits = productive.none {
            it.executable && it.estimatedCostSec + reserve <= remaining
        }
        val parkNow = runningCannotFinish || nothingElseFits

        return listOf(
            PhaseTask(
                phase = AutonomyPhase.PARK,
                priority = 65,
                executable = parkNow,
                estimatedCostSec = parkSec,
                canPreempt = false, // once we start parking we finish parking
                mandatory = true,
            ),
            fireTask,
            alignTask,
            collectTask,
            flowerTask,
            polishTask,
        )
    }

    /**
     * True when the policy wants the robot to score now — the hopper is full
     * or the clock has run down to where the score run no longer fits later.
     *
     * The StrategicPlanner makes the same call from the same inputs; the two
     * agreeing is what stops the phase machine and the planner from fighting
     * each other.
     */
    fun mustScore(state: PhaseState): Boolean {
        if (state.ballsHeld <= 0) return false
        if (state.ballsHeld >= maxBallsHeld) return true
        return state.remainingSec <= state.reserveSec + endgameMarginSec
    }
}
