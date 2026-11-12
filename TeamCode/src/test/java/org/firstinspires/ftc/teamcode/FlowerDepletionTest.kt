package org.firstinspires.ftc.teamcode

import kotlin.math.hypot
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * JVM tests for the flower depletion model and the adaptive clustering.
 *
 * The problem the depletion model solves: a flower's track id is re-issued
 * every time the camera loses the flower and finds it again, and the flower
 * clusters are rebuilt every second. Neither can carry a cooldown, so the
 * robot used to drive straight back to the flower it had just emptied.
 *
 * The model therefore tracks the depletion by POSITION (a depleted patch),
 * and the tests check that:
 *
 *  * a pulled flower leaves the available set,
 *  * a re-identified flower (new track id) is STILL excluded,
 *  * pulling twice from the same bed grows the cooldown (diminishing returns),
 *  * the cooldown expires and the flower comes back,
 *  * the clustering epsilon adapts to how the flowers are spread out.
 */
class FlowerDepletionTest {

    private var nowSec = 0.0
    private val nowNanos: Long get() = (nowSec * 1e9).toLong()

    private fun flower(distCm: Double, bearingDeg: Double = 0.0) =
        Sighting("Flower", distCm, bearingDeg, 400.0, isBall = false)

    private fun feed(model: WorldModel, vararg sightings: Sighting) {
        model.update(sightings.toList(), 0.0, 0.0, 0.0, nowNanos)
    }

    @Test
    fun `a pulled flower leaves the available set`() {
        val m = WorldModel()
        feed(m, flower(100.0))
        val id = m.flowers().first().id
        assertEquals(1, m.availableFlowers(nowNanos).size)

        m.markFlowerPulled(id, nowNanos)
        assertTrue(m.availableFlowers(nowNanos).isEmpty())
        assertEquals(FlowerState.DEPLETED, m.flowerState(id, nowNanos))
    }

    @Test
    fun `a re-identified flower is still excluded because the spot is depleted`() {
        val m = WorldModel()
        feed(m, flower(100.0))
        val firstId = m.flowers().first().id
        m.markFlowerPulled(firstId, nowNanos)

        // the camera loses the flower (it expires) and finds it again — the
        // track id is different, so only the spatial patch can stop us
        nowSec += WM_ROBOT_MAX_AGE_SEC + 1.0
        feed(m)
        assertTrue("the old track should have expired", m.flowers().isEmpty())

        feed(m, flower(100.0))
        val secondId = m.flowers().first().id
        assertTrue("the re-identified flower should have a new id", secondId != firstId)
        assertTrue(
            "the re-identified flower must stay excluded",
            m.availableFlowers(nowNanos).isEmpty(),
        )
    }

    @Test
    fun `pulling twice from the same bed grows the cooldown`() {
        val m = WorldModel()
        feed(m, flower(100.0))
        val firstId = m.flowers().first().id
        m.markFlowerPulled(firstId, nowNanos)

        // the camera loses the flower and finds it again — a brand-new track
        // id on the very same spot — and the robot pulls it a second time
        nowSec += WM_ROBOT_MAX_AGE_SEC + 1.0
        feed(m)
        feed(m, flower(100.0))
        val secondId = m.flowers().first().id
        assertTrue("the flower should have been re-identified", secondId != firstId)
        m.markFlowerPulled(secondId, nowNanos)

        // the per-track record starts fresh (it is a new flower), but the
        // SPOT remembers: its cooldown grew with the second pull
        val patch = m.nearestDepletedPatch(100.0, 0.0, nowNanos)
        assertNotNull(patch)
        assertEquals("both pulls count against the same spot", 2, patch!!.pullCount)
        assertTrue(
            "the spot cooldown should have grown beyond ${patch.cooldownRemainingSec(nowNanos)}",
            patch.cooldownRemainingSec(nowNanos) > FLOWER_COOLDOWN_BASE_SEC + FLOWER_COOLDOWN_PER_PULL_SEC,
        )
        assertTrue(
            "the flower must still be excluded",
            m.availableFlowers(nowNanos).isEmpty(),
        )
    }

    @Test
    fun `the cooldown expires and the flower comes back`() {
        val m = WorldModel()
        feed(m, flower(100.0))
        val id = m.flowers().first().id
        m.markFlowerPulled(id, nowNanos)

        val fullAfter = FLOWER_COOLDOWN_BASE_SEC +
            FLOWER_COOLDOWN_PER_PULL_SEC + FLOWER_REGEN_SEC + 1.0
        nowSec += fullAfter
        assertEquals(FlowerState.FULL, m.flowerState(id, nowNanos))
        assertEquals(1, m.availableFlowers(nowNanos).size)
    }

    @Test
    fun `the cluster stops being available once its bed is emptied`() {
        val m = WorldModel()
        // two flowers 60cm apart: two tracks, one bed
        feed(m, flower(60.0), flower(120.0))
        m.forceClusterRebuild(nowNanos)
        assertTrue("expected one cluster", m.flowerClusters().size == 1)

        val id = m.flowers().first().id
        m.markFlowerPulled(id, nowNanos)
        m.forceClusterRebuild(nowNanos)
        assertTrue(
            "the depleted bed must not be offered",
            m.availableFlowerClusters(nowNanos).isEmpty(),
        )
    }

    @Test
    fun `pulling from one bed does not block a different bed`() {
        val m = WorldModel()
        // two beds far apart: 60cm and 300cm
        feed(m, flower(60.0), flower(300.0))
        m.forceClusterRebuild(nowNanos)
        assertEquals(2, m.flowerClusters().size)

        val nearId = m.flowers().minByOrNull { it.fieldXcm }!!.id
        m.markFlowerPulled(nearId, nowNanos)
        m.forceClusterRebuild(nowNanos)

        val remaining = m.availableFlowerClusters(nowNanos)
        assertEquals("the far bed must still be available", 1, remaining.size)
        assertTrue(remaining.first().centerXcm > 200.0)
    }

    @Test
    fun `the clustering epsilon adapts to how spread out the flowers are`() {
        val m = WorldModel()

        // a tight bed: two tracks 45cm apart (just past the tracker's
        // association radius) -> a mean nearest-neighbour distance of 45cm
        feed(m, flower(60.0), flower(105.0))
        val denseEps = m.adaptiveClusterEps(m.flowers())
        assertEquals(
            FLOWER_CLUSTER_EPS_KNN_FACTOR * 45.0,
            denseEps,
            1e-6,
        )

        // a scattered field: two tracks 240cm apart -> the epsilon hits its
        // upper clamp
        nowSec += 1.0
        feed(m, flower(60.0), flower(300.0))
        val sparseEps = m.adaptiveClusterEps(m.flowers())
        assertEquals(FLOWER_CLUSTER_EPS_MAX_CM, sparseEps, 1e-9)
        assertTrue("the sparse field must use a bigger epsilon", sparseEps > denseEps)
    }

    @Test
    fun `the epsilon always stays inside its clamps`() {
        val m = WorldModel()
        feed(m, flower(60.0), flower(105.0))
        val eps = m.adaptiveClusterEps(m.flowers())
        assertTrue("eps $eps is below the minimum", eps >= FLOWER_CLUSTER_EPS_MIN_CM)
        assertTrue("eps $eps is above the maximum", eps <= FLOWER_CLUSTER_EPS_MAX_CM)
    }

    @Test
    fun `close flowers form one cluster and far flowers form two`() {
        val m = WorldModel()
        feed(m, flower(60.0), flower(120.0)) // 60cm apart -> one bed
        m.forceClusterRebuild(nowNanos)
        assertEquals("close flowers belong to one cluster", 1, m.flowerClusters().size)

        nowSec += 1.0
        feed(m, flower(60.0), flower(120.0), flower(300.0)) // one far flower
        m.forceClusterRebuild(nowNanos)
        assertEquals("the far flower is its own cluster", 2, m.flowerClusters().size)
    }

    @Test
    fun `a forced rebuild picks up a newly emptied bed immediately`() {
        val m = WorldModel()
        feed(m, flower(100.0))
        m.forceClusterRebuild(nowNanos)
        val before = m.flowerClusterRevision

        val id = m.flowers().first().id
        m.markFlowerPulled(id, nowNanos)
        m.forceClusterRebuild(nowNanos)

        assertTrue(
            "the revision should have moved on",
            m.flowerClusterRevision > before,
        )
        assertTrue(m.availableFlowerClusters(nowNanos).isEmpty())
    }

    @Test
    fun `the nearest depleted patch is reported`() {
        val m = WorldModel()
        feed(m, flower(100.0))
        val id = m.flowers().first().id
        m.markFlowerPulled(id, nowNanos)

        val patch = m.nearestDepletedPatch(100.0, 0.0, nowNanos)
        assertNotNull(patch)
        assertTrue(hypot(patch!!.xcm - 100.0, patch.ycm) < 1.0)
        assertTrue(m.isDepletedSpot(100.0, 0.0, nowNanos))
        assertFalse(m.isDepletedSpot(0.0, 0.0, nowNanos))
    }

    @Test
    fun `a depleted spot stops counting once its cooldown expires`() {
        val m = WorldModel()
        feed(m, flower(100.0))
        val id = m.flowers().first().id
        m.markFlowerPulled(id, nowNanos)

        nowSec += FLOWER_COOLDOWN_BASE_SEC + FLOWER_COOLDOWN_PER_PULL_SEC + 1.0
        assertFalse(m.isDepletedSpot(100.0, 0.0, nowNanos))
        assertNull(m.nearestDepletedPatch(100.0, 0.0, nowNanos))
    }

    @Test
    fun `the best cluster is the one with the most nectar that is still reachable`() {
        val m = WorldModel()
        // a two-flower bed near the robot and a single flower far away
        feed(m, flower(60.0), flower(120.0), flower(300.0))
        m.forceClusterRebuild(nowNanos)

        val best = m.bestFlowerCluster(0.0, 0.0, 45.0, 0, nowNanos)
        assertNotNull(best)
        assertEquals("the near bed has the most nectar", 2, best!!.flowerCount)
    }

    @Test
    fun `a cluster that cannot be reached in time is skipped`() {
        val m = WorldModel()
        // the near bed has more nectar but costs 10s per flower to pull, so
        // with only 30s left it does not fit — the far single flower does
        feed(m, flower(60.0), flower(120.0), flower(300.0))
        m.forceClusterRebuild(nowNanos)

        val best = m.bestFlowerCluster(0.0, 0.0, 30.0, 0, nowNanos)
        assertNotNull(best)
        assertEquals("only the single-flower bed fits", 1, best!!.flowerCount)
    }
}
