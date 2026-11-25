package org.firstinspires.ftc.teamcode

/**
 * Pure-Kotlin hopper inventory tracker — no Android imports, JVM-testable.
 *
 * Splits what LEFT the hopper (loader-ejected / attempted) from what actually
 * SCORED (confirmed flywheel shots), so a missed shot:
 *   * never inflates [totalScored] (counted only when confirmed), and
 *   * never leaves phantom retained inventory (once ejected, [held] drops
 *     regardless of whether the ball scored).
 *
 * The autonomous score loop owns one instance for the whole match so physical
 * inventory persists across collect → align → fire cycles instead of being
 * re-derived from `collected` each cycle.
 */
class HopperInventory(private val capacity: Int = MAX_BALLS_HELD) {

    /** Balls physically in the hopper right now. */
    var held: Int = 0
        private set

    /** Loader-ejected balls this match (left the hopper, scored or not). */
    var attempted: Int = 0
        private set

    /** Shots confirmed by the flywheel RPM dip — actually scored. */
    var confirmed: Int = 0
        private set

    /** Match score: confirmed shots only. */
    val totalScored: Int
        get() = confirmed

    /** Slots still free in the hopper. */
    fun remainingCapacity(): Int = capacity - held.coerceAtMost(capacity)

    fun isFull(): Boolean = held >= capacity
    fun isEmpty(): Boolean = held <= 0

    /**
     * Add [n] collected balls, capped by remaining capacity.
     * Returns how many were actually added.
     */
    fun addCollected(n: Int): Int {
        val add = n.coerceIn(0, remainingCapacity())
        held += add
        return add
    }

    /** Record a successful flower pull (+1 ball). False when the hopper is full. */
    fun addFlower(): Boolean {
        if (isFull()) return false
        held++
        return true
    }

    /**
     * Eject [n] balls from the hopper (one loader stroke each). Capped by [held].
     * Returns how many were actually ejected. [held] drops even for misses.
     */
    fun eject(n: Int): Int {
        val ejected = n.coerceIn(0, held)
        held -= ejected
        attempted += ejected
        return ejected
    }

    /** Mark [n] of the ejected balls as confirmed (scored). Never exceeds [attempted]. */
    fun confirm(n: Int) {
        confirmed = (confirmed + n.coerceAtLeast(0)).coerceAtMost(attempted)
    }
}
