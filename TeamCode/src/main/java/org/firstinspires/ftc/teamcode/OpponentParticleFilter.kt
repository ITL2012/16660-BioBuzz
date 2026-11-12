package org.firstinspires.ftc.teamcode

import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.random.Random

/**
 * ============ OPPONENT PARTICLE FILTER — robust opponent tracking ============
 *
 * A single point estimate of where the opponent is is brittle: one bad
 * detection yanks the estimate across the field, and the opponent vanishing
 * for a second leaves the stale estimate where it was. Both mistakes are
 * expensive here — the dodge logic and the planner's "an opponent is closer
 * to this ball" discount both read the opponent's position.
 *
 * A particle filter keeps a cloud of weighted hypotheses instead:
 *
 *  1. PREDICT — every particle is moved forward by its own velocity plus
 *     process noise. The cloud spreads out while the opponent is unseen,
 *     which is exactly the uncertainty we want to represent.
 *  2. UPDATE — each particle's weight is multiplied by the likelihood of
 *     the measurement under that hypothesis. A particle near the detection
 *     survives; a particle across the field is killed.
 *  3. RESAMPLE — when the effective sample size (the reciprocal of the sum
 *     of squared weights) drops too low, the cloud has collapsed onto one
 *     hypothesis and can no longer recover. Systematic resampling draws a
 *     fresh cloud from the weighted particles, and a fraction of the new
 *     particles is re-injected around the current estimate so the filter
 *     keeps some diversity.
 *
 * The reported position is the weighted mean of the cloud — smooth, and far
 * less jumpy than the raw detection — and the reported velocity is the
 * weighted mean of the particle velocities, which is a much better
 * extrapolation for the dodge lookahead than a finite difference.
 *
 * Pure Kotlin and allocation-light: the particle array is reused, so a
 * steady-state update does not allocate.
 */
class OpponentParticleFilter(
    val particleCount: Int = OPP_PARTICLE_COUNT,
    private val random: Random = Random.Default,
) {
    /** One hypothesis about the opponent's state. */
    class Particle(
        var x: Double,
        var y: Double,
        var vx: Double,
        var vy: Double,
        var weight: Double,
    )

    private val particles = Array(particleCount) { Particle(0.0, 0.0, 0.0, 0.0, 1.0 / particleCount) }

    /** True once the filter has been seeded with an observation. */
    var initialized: Boolean = false
        private set

    /** Number of resamples performed (diagnostics). */
    var resampleCount: Int = 0
        private set

    /**
     * Seeds the cloud around the first observation.
     */
    fun initialize(x: Double, y: Double, vx: Double = 0.0, vy: Double = 0.0) {
        for (p in particles) {
            p.x = x + gaussian() * OPP_PARTICLE_POS_NOISE_CM
            p.y = y + gaussian() * OPP_PARTICLE_POS_NOISE_CM
            p.vx = vx
            p.vy = vy
            p.weight = 1.0 / particleCount
        }
        initialized = true
    }

    /**
     * Advances the cloud by [dtSec] using a constant-velocity model with
     * process noise that grows with sqrt(dt).
     */
    fun predict(dtSec: Double) {
        if (!initialized || dtSec <= 0.0) return
        val posSigma = OPP_PARTICLE_POS_NOISE_CM * Math.sqrt(dtSec)
        val velSigma = OPP_PARTICLE_VEL_NOISE_CM_PER_SEC * Math.sqrt(dtSec)
        for (p in particles) {
            p.x += p.vx * dtSec + gaussian() * posSigma
            p.y += p.vy * dtSec + gaussian() * posSigma
            p.vx += gaussian() * velSigma
            p.vy += gaussian() * velSigma
        }
    }

    /**
     * Folds an observation into the cloud and reweights it.
     *
     * @param obsX observed position, field cm
     * @param obsY observed position, field cm
     * @param measNoiseCm sigma of the measurement noise, cm
     */
    fun update(obsX: Double, obsY: Double, measNoiseCm: Double = OPP_PARTICLE_MEAS_NOISE_CM) {
        if (!initialized) {
            initialize(obsX, obsY)
            return
        }
        val sigma = max(measNoiseCm, 1.0)
        val inv2s2 = 1.0 / (2.0 * sigma * sigma)
        var total = 0.0
        for (p in particles) {
            val dx = p.x - obsX
            val dy = p.y - obsY
            val likelihood = exp(-(dx * dx + dy * dy) * inv2s2)
            p.weight *= likelihood
            total += p.weight
        }
        if (total <= 1e-12 || !total.isFinite()) {
            // Every particle died (bad measurement) — restart around it.
            initialize(obsX, obsY)
            return
        }
        for (p in particles) p.weight /= total
        resampleIfDegenerate()
    }

    /**
     * Tells the cloud how fast the opponent is actually moving.
     *
     * The cloud's job is to hold a set of POSITION hypotheses; the velocity
     * comes from the track's own estimator (which sees the robot's motion
     * compensated, so it is far steadier than anything the cloud could
     * derive from a single noisy sighting). Carrying that velocity on the
     * particles buys two things:
     *
     *  * the predict step carries the cloud ALONG WITH the opponent instead
     *    of leaving it a frame behind, so the likelihood keeps comparing the
     *    cloud against where the opponent actually IS, not where it was,
     *  * [estimateVelocity] can report the velocity with the cloud's spread
     *    attached, which says how much the robot should trust it.
     *
     * The hint is blended in rather than assigned, so the process noise
     * keeps a spread around it — that spread is the uncertainty.
     */
    fun setVelocityHint(vx: Double, vy: Double) {
        if (!initialized) return
        val blend = OPP_PARTICLE_VEL_HINT_ALPHA
        for (p in particles) {
            p.vx += blend * (vx - p.vx)
            p.vy += blend * (vy - p.vy)
        }
    }

    /**
     * Resamples when the effective sample size falls below the threshold.
     */
    private fun resampleIfDegenerate() {
        var sumSq = 0.0
        for (p in particles) sumSq += p.weight * p.weight
        val ess = if (sumSq > 0.0) 1.0 / sumSq else 0.0
        if (ess >= particleCount * OPP_PARTICLE_RESAMPLE_ESS_RATIO) return

        resampleCount++
        // Systematic resampling: one uniform draw, then walk the CDF. Low
        // variance and O(n).
        val step = 1.0 / particleCount
        var u = random.nextDouble() * step
        val carry = ArrayList<Particle>(particleCount)
        var index = 0
        for (i in 0 until particleCount) {
            while (index < particleCount && u > particles[index].weight) {
                u -= particles[index].weight
                index++
            }
            val src = particles[min(index, particleCount - 1)]
            carry.add(Particle(src.x, src.y, src.vx, src.vy, 1.0 / particleCount))
        }
        for (i in 0 until particleCount) particles[i] = carry[i]

        // Re-inject diversity: spread a fraction of the cloud around the
        // current estimate so the filter can jump to a new hypothesis.
        val (ex, ey) = estimatePosition()
        val reinject = (particleCount * OPP_PARTICLE_REINJECT_RATIO).toInt().coerceAtLeast(1)
        for (i in 0 until reinject) {
            val p = particles[particleCount - 1 - i]
            p.x = ex + gaussian() * OPP_PARTICLE_POS_NOISE_CM * 1.5
            p.y = ey + gaussian() * OPP_PARTICLE_POS_NOISE_CM * 1.5
        }
    }

    /** Weighted mean position of the cloud (field cm). */
    fun estimatePosition(): Pair<Double, Double> {
        var sx = 0.0
        var sy = 0.0
        var sw = 0.0
        for (p in particles) {
            sx += p.x * p.weight
            sy += p.y * p.weight
            sw += p.weight
        }
        if (sw <= 0.0) return particles[0].x to particles[0].y
        return sx / sw to sy / sw
    }

    /** Weighted mean velocity of the cloud (cm/s). */
    fun estimateVelocity(): Pair<Double, Double> {
        var svx = 0.0
        var svy = 0.0
        var sw = 0.0
        for (p in particles) {
            svx += p.vx * p.weight
            svy += p.vy * p.weight
            sw += p.weight
        }
        if (sw <= 0.0) return 0.0 to 0.0
        return svx / sw to svy / sw
    }

    /**
     * Effective sample size — a number in [1, particleCount]. Small values
     * mean the cloud has collapsed onto one hypothesis.
     */
    fun effectiveSampleSize(): Double {
        var sumSq = 0.0
        for (p in particles) sumSq += p.weight * p.weight
        return if (sumSq > 0.0) 1.0 / sumSq else 0.0
    }

    /** Spread of the cloud (weighted RMS radius), cm. */
    fun positionSpreadCm(): Double {
        val (ex, ey) = estimatePosition()
        var s = 0.0
        var sw = 0.0
        for (p in particles) {
            val dx = p.x - ex
            val dy = p.y - ey
            s += (dx * dx + dy * dy) * p.weight
            sw += p.weight
        }
        if (sw <= 0.0) return 0.0
        return Math.sqrt(s / sw)
    }

    /**
     * Standard normal sample (Box-Muller, cached second value).
     */
    private var spareGaussian: Double? = null

    private fun gaussian(): Double {
        spareGaussian?.let {
            spareGaussian = null
            return it
        }
        var u1: Double
        var u2: Double
        do {
            u1 = random.nextDouble()
        } while (u1 <= 1e-12)
        u2 = random.nextDouble()
        val mag = Math.sqrt(-2.0 * Math.log(u1))
        spareGaussian = mag * Math.sin(2.0 * Math.PI * u2)
        return mag * Math.cos(2.0 * Math.PI * u2)
    }
}

/**
 * A tiny helper the WorldModel uses to keep one particle filter per tracked
 * opponent, so filters are created lazily and die with their track.
 */
class OpponentParticleBank {
    private val filters = mutableMapOf<Int, OpponentParticleFilter>()

    fun filterFor(trackId: Int): OpponentParticleFilter =
        filters.getOrPut(trackId) { OpponentParticleFilter() }

    fun remove(trackId: Int) {
        filters.remove(trackId)
    }

    fun prune(validTrackIds: Set<Int>) {
        filters.keys.retainAll(validTrackIds)
    }

    fun clear() {
        filters.clear()
    }

    fun size(): Int = filters.size
}
