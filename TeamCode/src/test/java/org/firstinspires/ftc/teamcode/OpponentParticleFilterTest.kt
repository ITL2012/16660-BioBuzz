package org.firstinspires.ftc.teamcode

import kotlin.math.abs
import kotlin.random.Random
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * JVM tests for the opponent particle filter.
 *
 * The filter exists because a single point estimate of the opponent is
 * brittle: one bad detection yanks it across the field, and a lost track
 * leaves it stale. The tests check the three properties that matter for the
 * dodge logic and the planner's opponent discount:
 *
 *  * it converges on the observation,
 *  * a single outlier barely moves it,
 *  * its velocity estimate tracks a constant-velocity target.
 */
class OpponentParticleFilterTest {

    @Test
    fun `the estimate converges on the observation`() {
        val filter = OpponentParticleFilter(particleCount = 200, random = Random(42))
        filter.initialize(100.0, 50.0)
        repeat(20) { filter.update(100.0, 50.0) }
        val (x, y) = filter.estimatePosition()
        assertEquals(100.0, x, 5.0)
        assertEquals(50.0, y, 5.0)
    }

    @Test
    fun `a single outlier does not drag the estimate across the field`() {
        val filter = OpponentParticleFilter(particleCount = 200, random = Random(7))
        filter.initialize(100.0, 0.0)
        repeat(10) { filter.update(100.0, 0.0) }
        val before = filter.estimatePosition().first
        filter.update(300.0, 0.0) // a bogus detection 200cm away
        val after = filter.estimatePosition().first
        assertTrue(
            "the estimate jumped from $before to $after",
            abs(after - before) < 30.0,
        )
    }

    @Test
    fun `the velocity estimate follows the measured velocity hint`() {
        val filter = OpponentParticleFilter(particleCount = 300, random = Random(3))
        filter.initialize(0.0, 0.0)
        // the track's own estimator says the opponent is moving at 20 cm/s
        repeat(10) {
            filter.setVelocityHint(20.0, 0.0)
            filter.predict(0.1)
        }
        val (vx, _) = filter.estimateVelocity()
        assertEquals(20.0, vx, 3.0)
    }

    @Test
    fun `the cloud follows the opponent between sightings`() {
        val filter = OpponentParticleFilter(particleCount = 300, random = Random(15))
        filter.initialize(0.0, 0.0)
        // the hint is blended in every frame, so it converges quickly
        repeat(10) { filter.setVelocityHint(20.0, 0.0) }
        filter.predict(1.0)
        val (x, _) = filter.estimatePosition()
        assertEquals("the cloud should have moved with the opponent", 20.0, x, 5.0)
    }

    @Test
    fun `the effective sample size never exceeds the particle count`() {
        val filter = OpponentParticleFilter(particleCount = 64, random = Random(11))
        filter.initialize(0.0, 0.0)
        repeat(10) { filter.update(0.0, 0.0) }
        val ess = filter.effectiveSampleSize()
        assertTrue("ESS $ess is not in (0, 64]", ess > 0.0 && ess <= 64.0 + 1e-9)
    }

    @Test
    fun `a collapsed cloud triggers a resample`() {
        val filter = OpponentParticleFilter(particleCount = 40, random = Random(5))
        filter.initialize(0.0, 0.0)
        // a measurement a few sigmas away kills most of the particles, which
        // drives the effective sample size down and forces a resample
        var resamples = 0
        for (attempt in 1..5) {
            filter.update(3.0 * OPP_PARTICLE_MEAS_NOISE_CM, 0.0)
            resamples = filter.resampleCount
            if (resamples > 0) break
        }
        assertTrue("expected at least one resample", resamples >= 1)
    }

    @Test
    fun `an impossible measurement restarts the filter around it`() {
        val filter = OpponentParticleFilter(particleCount = 40, random = Random(9))
        filter.initialize(0.0, 0.0)
        // every particle's likelihood under this measurement is numerically
        // zero: the filter has to restart rather than believe the cloud
        filter.update(500.0, 500.0)
        assertTrue(filter.initialized)
        val (x, y) = filter.estimatePosition()
        assertEquals(500.0, x, 30.0)
        assertEquals(500.0, y, 30.0)
    }

    @Test
    fun `the cloud spreads while the opponent is unseen`() {
        val filter = OpponentParticleFilter(particleCount = 200, random = Random(13))
        filter.initialize(0.0, 0.0, vx = 10.0, vy = 0.0)
        val spreadBefore = filter.positionSpreadCm()
        repeat(10) { filter.predict(0.1) }
        val spreadAfter = filter.positionSpreadCm()
        assertTrue(
            "the cloud did not spread ($spreadBefore -> $spreadAfter)",
            spreadAfter > spreadBefore,
        )
    }

    @Test
    fun `an uninitialized filter seeds itself from the first observation`() {
        val filter = OpponentParticleFilter(particleCount = 50, random = Random(17))
        assertTrue(!filter.initialized)
        filter.update(75.0, 25.0)
        assertTrue(filter.initialized)
        val (x, y) = filter.estimatePosition()
        assertEquals(75.0, x, 10.0)
        assertEquals(25.0, y, 10.0)
    }

    @Test
    fun `the bank creates filters lazily and prunes dead tracks`() {
        val bank = OpponentParticleBank()
        assertEquals(0, bank.size())
        bank.filterFor(1)
        bank.filterFor(2)
        assertEquals(2, bank.size())
        bank.prune(setOf(1))
        assertEquals(1, bank.size())
        bank.remove(1)
        assertEquals(0, bank.size())
    }

    @Test
    fun `the bank hands back the same filter for the same track`() {
        val bank = OpponentParticleBank()
        val a = bank.filterFor(7)
        val b = bank.filterFor(7)
        assertTrue(a === b)
    }
}
