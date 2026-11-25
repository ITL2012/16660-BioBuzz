package org.firstinspires.ftc.teamcode

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * JVM tests for [HopperInventory] — the persistent hopper inventory tracker.
 * Pure Kotlin, no Android imports, runs on the JVM directly.
 */
class HopperInventoryTest {

    @Test
    fun `starts empty with nothing scored`() {
        val inv = HopperInventory()
        assertEquals(0, inv.held)
        assertEquals(0, inv.attempted)
        assertEquals(0, inv.confirmed)
        assertEquals(0, inv.totalScored)
        assertTrue(inv.isEmpty())
        assertFalse(inv.isFull())
        assertEquals(MAX_BALLS_HELD, inv.remainingCapacity())
    }

    @Test
    fun `addCollected is capped by remaining capacity`() {
        val inv = HopperInventory()
        assertEquals(4, inv.addCollected(4)) // hopper now full
        assertEquals(MAX_BALLS_HELD, inv.held)
        assertTrue(inv.isFull())
        assertEquals(0, inv.remainingCapacity())
        // further collection is rejected
        assertEquals(0, inv.addCollected(2))
        assertEquals(MAX_BALLS_HELD, inv.held)
    }

    @Test
    fun `collection caps at capacity across cycles`() {
        val inv = HopperInventory()
        inv.addCollected(3) // held 3
        inv.addCollected(3) // only 1 slot remains
        assertEquals(MAX_BALLS_HELD, inv.held)
        assertEquals(0, inv.remainingCapacity())
    }

    @Test
    fun `flower adds one ball and is rejected when full`() {
        val inv = HopperInventory()
        assertTrue(inv.addFlower())
        assertEquals(1, inv.held)
        // fill the rest
        inv.addCollected(MAX_BALLS_HELD - 1)
        assertTrue(inv.isFull())
        assertFalse(inv.addFlower()) // no room
        assertEquals(MAX_BALLS_HELD, inv.held)
    }

    @Test
    fun `inventory persists across an align failure`() {
        val inv = HopperInventory()
        // collect a full load
        inv.addCollected(MAX_BALLS_HELD)
        assertEquals(MAX_BALLS_HELD, inv.held)
        // align fails -> fire never happens, inventory untouched
        assertEquals(MAX_BALLS_HELD, inv.held)
        assertEquals(0, inv.remainingCapacity())
        // nothing can be collected until inventory is spent
        assertEquals(0, inv.addCollected(1))
    }

    @Test
    fun `attempted and confirmed are tracked separately`() {
        val inv = HopperInventory()
        inv.addCollected(4)
        val ejected = inv.eject(inv.held) // attempt to fire all 4
        assertEquals(4, ejected)
        assertEquals(0, inv.held)
        assertEquals(4, inv.attempted)
        // only 2 of the 4 were confirmed (2 missed)
        inv.confirm(2)
        assertEquals(2, inv.confirmed)
        assertEquals(2, inv.totalScored)
        assertEquals(4, inv.attempted) // attempts are not revised by misses
        // held stays 0: misses do not put balls back
        assertEquals(0, inv.held)
    }

    @Test
    fun `missed shots create no phantom retained inventory and score nothing`() {
        val inv = HopperInventory()
        inv.addCollected(4)
        inv.eject(4) // all 4 attempted
        inv.confirm(0) // none confirmed: 4 misses
        assertEquals(0, inv.held)
        assertEquals(4, inv.attempted)
        assertEquals(0, inv.confirmed)
        assertEquals(0, inv.totalScored)
    }

    @Test
    fun `eject is capped by held`() {
        val inv = HopperInventory()
        inv.addCollected(2)
        assertEquals(2, inv.eject(5)) // only 2 present
        assertEquals(0, inv.held)
        assertEquals(2, inv.attempted)
    }

    @Test
    fun `confirm never exceeds attempted`() {
        val inv = HopperInventory()
        inv.addCollected(1)
        inv.eject(1) // attempted 1
        inv.confirm(5) // clamped
        assertEquals(1, inv.confirmed)
        assertEquals(1, inv.totalScored)
        assertEquals(1, inv.attempted)
    }

    @Test
    fun `partial fire keeps the remainder for the next cycle`() {
        val inv = HopperInventory()
        inv.addCollected(4)
        val ejected = inv.eject(2) // only 2 fired before time ran out / target lost
        assertEquals(2, ejected)
        assertEquals(2, inv.held) // 2 remain for a later cycle
        inv.confirm(2)
        assertEquals(2, inv.totalScored)
    }
}
