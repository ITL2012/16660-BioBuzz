package org.firstinspires.ftc.teamcode

/** Tracks proximity-settle pickup confirmation independently of robot hardware. */
internal class SelfPlayPickupConfirmation(private val settleMs: Double) {
    private val confirmedTargetIds = mutableSetOf<Int>()
    private var approachTargetId: Int? = null
    private var closeSinceNanos = 0L

    /** Clears only the current approach; a confirmed track stays confirmed. */
    fun resetApproach() {
        approachTargetId = null
        closeSinceNanos = 0L
    }

    /** Returns true exactly once, when [targetId] completes a fresh settle interval. */
    fun update(targetId: Int, closeEnough: Boolean, nowNanos: Long): Boolean {
        if (targetId in confirmedTargetIds) return false
        if (approachTargetId != targetId) {
            approachTargetId = targetId
            closeSinceNanos = 0L
        }
        if (!closeEnough) {
            closeSinceNanos = 0L
            return false
        }
        if (closeSinceNanos == 0L) {
            closeSinceNanos = nowNanos
            return false
        }
        if ((nowNanos - closeSinceNanos) / 1e6 < settleMs) return false

        confirmedTargetIds.add(targetId)
        closeSinceNanos = 0L
        return true
    }
}

internal enum class FlowerPullEvent { NONE, START_REVERSE, COMPLETE }

/** Non-blocking SelfPlay flower pull; the deadman remains responsive between ticks. */
internal class SelfPlayFlowerPull(private val reverseDurationNanos: Long) {
    enum class Stage { IDLE, REVERSING, COMPLETE }

    var targetId: Int? = null
        private set
    var stage = Stage.IDLE
        private set
    private var reverseUntilNanos = 0L

    val reversing: Boolean get() = stage == Stage.REVERSING

    fun reset() {
        targetId = null
        stage = Stage.IDLE
        reverseUntilNanos = 0L
    }

    fun update(newTargetId: Int, inRange: Boolean, nowNanos: Long): FlowerPullEvent {
        if (targetId != newTargetId) {
            reset()
            targetId = newTargetId
        }
        return when (stage) {
            Stage.IDLE -> {
                if (!inRange) return FlowerPullEvent.NONE
                stage = Stage.REVERSING
                reverseUntilNanos = nowNanos + reverseDurationNanos
                FlowerPullEvent.START_REVERSE
            }
            Stage.REVERSING -> {
                if (nowNanos < reverseUntilNanos) return FlowerPullEvent.NONE
                stage = Stage.COMPLETE
                FlowerPullEvent.COMPLETE
            }
            Stage.COMPLETE -> FlowerPullEvent.NONE
        }
    }
}
