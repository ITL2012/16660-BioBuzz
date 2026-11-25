package org.firstinspires.ftc.teamcode

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class SelfPlayBehaviorsTest {
    @Test
    fun `one target confirms only once even when it remains nearby`() {
        val pickup = SelfPlayPickupConfirmation(settleMs = 400.0)
        assertFalse(pickup.update(7, closeEnough = true, nowNanos = 1_000_000_000L))
        assertFalse(pickup.update(7, closeEnough = true, nowNanos = 1_200_000_000L))
        assertTrue(pickup.update(7, closeEnough = true, nowNanos = 1_400_000_000L))
        assertFalse(pickup.update(7, closeEnough = true, nowNanos = 1_800_000_000L))
        assertFalse(pickup.update(7, closeEnough = true, nowNanos = 3_000_000_000L))
    }

    @Test
    fun `switching targets requires a fresh settle interval`() {
        val pickup = SelfPlayPickupConfirmation(settleMs = 400.0)
        pickup.update(1, closeEnough = true, nowNanos = 1_000_000_000L)
        pickup.update(1, closeEnough = true, nowNanos = 1_300_000_000L)
        assertFalse(pickup.update(2, closeEnough = true, nowNanos = 1_350_000_000L))
        assertFalse(pickup.update(2, closeEnough = true, nowNanos = 1_700_000_000L))
        assertTrue(pickup.update(2, closeEnough = true, nowNanos = 1_750_000_000L))
    }

    @Test
    fun `leaving proximity resets settle timing`() {
        val pickup = SelfPlayPickupConfirmation(settleMs = 400.0)
        pickup.update(1, closeEnough = true, nowNanos = 1_000_000_000L)
        pickup.update(1, closeEnough = false, nowNanos = 1_300_000_000L)
        assertFalse(pickup.update(1, closeEnough = true, nowNanos = 1_350_000_000L))
        assertFalse(pickup.update(1, closeEnough = true, nowNanos = 1_700_000_000L))
        assertTrue(pickup.update(1, closeEnough = true, nowNanos = 1_750_000_000L))
    }

    @Test
    fun `flower pull advances without blocking and completes once`() {
        val pull = SelfPlayFlowerPull(reverseDurationNanos = 1_000_000_000L)
        assertEquals(FlowerPullEvent.NONE, pull.update(4, inRange = false, nowNanos = 0L))
        assertEquals(FlowerPullEvent.START_REVERSE, pull.update(4, inRange = true, nowNanos = 10L))
        assertTrue(pull.reversing)
        assertEquals(FlowerPullEvent.NONE, pull.update(4, inRange = false, nowNanos = 500_000_000L))
        assertEquals(FlowerPullEvent.COMPLETE, pull.update(4, inRange = false, nowNanos = 1_000_000_010L))
        assertEquals(FlowerPullEvent.NONE, pull.update(4, inRange = true, nowNanos = 2_000_000_000L))
    }

    @Test
    fun `deadman cancellation permits a fresh flower pull`() {
        val pull = SelfPlayFlowerPull(reverseDurationNanos = 1_000_000_000L)
        pull.update(4, inRange = true, nowNanos = 0L)
        pull.reset()
        assertFalse(pull.reversing)
        assertEquals(FlowerPullEvent.START_REVERSE, pull.update(4, inRange = true, nowNanos = 10L))
    }
}
