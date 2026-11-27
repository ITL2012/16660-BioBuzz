package org.firstinspires.ftc.teamcode.vision

import com.acmerobotics.roadrunner.Pose2d
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

// ============================================================================
// SECTION 1: RANSAC robust static-scene estimation (inlier/outlier classification)
// ============================================================================

/**
 * RANSAC fitting of a 2D similarity transform (rotation + uniform scale +
 * translation) to a set of feature correspondences.
 *
 * WHY a similarity transform: a camera viewing a dominant plane (ceiling for
 * the UP camera, wall for the FRONT camera) while translating and yawing on
 * the floor induces approximately a similarity transform on that plane's image
 * (a homography that degenerates to a similarity for small rotations and
 * roughly perpendicular views). Features that AGREE with the dominant
 * transformation belong to the stationary environment; features that
 * repeatedly disagree are moving objects (robots, people, game elements) or
 * bad optical-flow tracks.
 *
 * IMPORTANT: this detects features that *move consistently with the static
 * scene*, NOT features whose pixel coordinates stay constant. Static objects
 * sweep across the image as the robot moves.
 */
class StaticSceneEstimator {

    /** Result of one RANSAC fit over a frame's correspondences. */
    data class Result(
        /** Fit succeeded (enough inliers, sane scale). */
        val ok: Boolean,
        /** Rotation component (radians, image space, CCW positive). */
        val thetaRad: Double,
        /** Translation component (pixels). */
        val txPx: Double,
        val tyPx: Double,
        /** Uniform scale component (dimensionless). */
        val scale: Double,
        /** Indices (into the input arrays) of features consistent with the scene. */
        val inlierIndices: IntArray,
        val inlierRatio: Double,
    )

    /**
     * @param prev previous pixel positions (x0,y0,x1,y1,...)
     * @param cur  current pixel positions (same length)
     * @return robust scene-motion estimate with inlier classification
     */
    fun estimate(prev: DoubleArray, cur: DoubleArray): Result {
        val n = prev.size / 2
        if (n < VisionTuning.RANSAC_MIN_INLIERS) return failure()

        var bestInliers = IntArray(0)
        var bestCount = -1
        var bestTheta = 0.0
        var bestTx = 0.0
        var bestTy = 0.0
        var bestScale = 1.0
        val candidateInliers = IntArray(n)
        val thresholdSq = VisionTuning.RANSAC_INLIER_THRESHOLD_PX *
            VisionTuning.RANSAC_INLIER_THRESHOLD_PX

        for (iter in 0 until VisionTuning.RANSAC_ITERATIONS) {
            val i = pick(n, iter)
            val j = pick(n, iter + 7919)
            if (i == j) continue
            val m = fitTwo(prev, cur, i, j) ?: continue

            var count = 0
            for (k in 0 until n) {
                val px = prev[2 * k]
                val py = prev[2 * k + 1]
                val dx = cur[2 * k] - (m.a * px - m.b * py + m.tx)
                val dy = cur[2 * k + 1] - (m.b * px + m.a * py + m.ty)
                if (dx * dx + dy * dy <= thresholdSq) {
                    candidateInliers[count] = k
                    count++
                }
            }

            if (count > bestCount) {
                bestCount = count
                bestInliers = candidateInliers.copyOf(count)
                bestTheta = m.theta
                bestTx = m.tx
                bestTy = m.ty
                bestScale = m.scale
                if (bestCount == n) break
            }
        }

        val ratio = bestCount.toDouble() / n
        val ok = bestCount >= VisionTuning.RANSAC_MIN_INLIERS &&
            ratio >= VisionTuning.RANSAC_MIN_INLIER_RATIO &&
            abs(bestScale - 1.0) <= VisionTuning.RANSAC_MAX_SCALE_ERROR
        if (!ok) return failure()

        val refined = refit(prev, cur, bestInliers) ?: return failure()
        val finalInliers = IntArray(n)
        var count = 0
        for (k in 0 until n) {
            val px = prev[2 * k]
            val py = prev[2 * k + 1]
            val dx = cur[2 * k] - (refined.a * px - refined.b * py + refined.tx)
            val dy = cur[2 * k + 1] - (refined.b * px + refined.a * py + refined.ty)
            if (dx * dx + dy * dy <= thresholdSq) {
                finalInliers[count] = k
                count++
            }
        }

        return Result(
            ok = true,
            thetaRad = refined.theta,
            txPx = refined.tx,
            tyPx = refined.ty,
            scale = refined.scale,
            inlierIndices = finalInliers.copyOf(count),
            inlierRatio = count.toDouble() / n,
        )
    }

    private fun failure() = Result(
        ok = false, thetaRad = 0.0, txPx = 0.0, tyPx = 0.0, scale = 1.0,
        inlierIndices = IntArray(0), inlierRatio = 0.0,
    )

    /** Pseudo-random index spread over [0, n) — fast, stateless. */
    private fun pick(n: Int, seed: Int): Int {
        val x = (seed * 2654435761L).toInt()
        return ((x ushr 16) and 0x7FFFFFFF) % n
    }

    private data class Model(
        val theta: Double,
        val tx: Double,
        val ty: Double,
        val scale: Double,
        val a: Double,
        val b: Double,
    )

    /** Exact similarity fit from 2 correspondences. Returns null on degenerate geometry. */
    private fun fitTwo(prev: DoubleArray, cur: DoubleArray, i: Int, j: Int): Model? {
        val p1x = prev[2 * i]; val p1y = prev[2 * i + 1]
        val p2x = prev[2 * j]; val p2y = prev[2 * j + 1]
        val c1x = cur[2 * i];  val c1y = cur[2 * i + 1]
        val c2x = cur[2 * j];  val c2y = cur[2 * j + 1]

        val dpx = p2x - p1x
        val dpy = p2y - p1y
        val dcx = c2x - c1x
        val dcy = c2y - c1y

        val dpSq = dpx * dpx + dpy * dpy
        val dcSq = dcx * dcx + dcy * dcy
        if (dpSq < 1e-6 || dcSq < 1e-6) return null

        val scale = kotlin.math.sqrt(dcSq / dpSq)
        val theta = atan2(
            dpx * dcy - dpy * dcx,
            dpx * dcx + dpy * dcy,
        )
        if (!theta.isFinite()) return null

        val a = cos(theta) * scale
        val b = sin(theta) * scale
        val tx = c1x - (a * p1x - b * p1y)
        val ty = c1y - (b * p1x + a * p1y)
        return Model(theta, tx, ty, scale, a, b)
    }

    /** Least-squares similarity refit over the consensus set (Procrustes-style). */
    private fun refit(prev: DoubleArray, cur: DoubleArray, idx: IntArray): Model? {
        if (idx.size < 2) return null

        var mx = 0.0; var my = 0.0; var cx = 0.0; var cy = 0.0
        for (k in idx) {
            mx += prev[2 * k]; my += prev[2 * k + 1]
            cx += cur[2 * k];  cy += cur[2 * k + 1]
        }
        val n = idx.size
        mx /= n; my /= n; cx /= n; cy /= n

        var a = 0.0  // sum of (p . c) terms
        var b = 0.0  // sum of (p x c) terms
        var sp = 0.0 // sum of |p - centroid|^2
        for (k in idx) {
            val px = prev[2 * k] - mx
            val py = prev[2 * k + 1] - my
            val qx = cur[2 * k] - cx
            val qy = cur[2 * k + 1] - cy
            a += px * qx + py * qy
            b += px * qy - py * qx
            sp += px * px + py * py
        }
        if (sp < 1e-6) return null

        val scale = kotlin.math.sqrt((a * a + b * b) / (sp * sp))
        val theta = atan2(b, a)
        val rotScaleX = cos(theta) * scale
        val rotScaleY = sin(theta) * scale
        val tx = cx - (rotScaleX * mx - rotScaleY * my)
        val ty = cy - (rotScaleY * mx + rotScaleX * my)
        return Model(theta, tx, ty, scale, rotScaleX, rotScaleY)
    }

}

// ============================================================================
// SECTION 2: Image motion -> robot-frame motion (documented monocular assumptions)
// ============================================================================



/**
 * Converts a per-camera *image-space* scene motion (from RANSAC) into a
 * *robot-frame* relative motion delta, using the camera calibration.
 *
 * DOCUMENTED ASSUMPTIONS (first-order monocular estimator, not full 3D):
 *  1. The dominant observed features lie on a plane roughly perpendicular to
 *     the camera axis (ceiling for the UP camera, wall for the FRONT camera).
 *  2. The perpendicular distance to that plane is approximately constant and
 *     known from calibration ([CameraExtrinsics.sceneDistanceCm]). Translation
 *     accuracy is exactly as good as this value: pixel motion -> cm via
 *     d_cm = px * depth_cm / focal_px.
 *  3. Robot yaw maps to image rotation with a fixed sign and unit scale
 *     ([CameraExtrinsics.headingSign]): 1 rad of robot yaw = 1 rad of image
 *     scene rotation (exact for a planar scene under pure yaw).
 *  4. Camera roll/pitch and out-of-plane depth variation are neglected; the
 *     RANSAC scale sanity check rejects frames where these are badly violated.
 *
 * This provides RELATIVE motion (visual odometry). It cannot, by itself,
 * produce absolute field coordinates — a known map or known targets would be
 * required for that (see the existing AprilTag VisionSystem for known targets).
 */
class VisualMotionEstimator(private val calibration: CameraCalibration) {

    /**
     * @param ransac RANSAC similarity fit of the frame pair (image space)
     * @param timestampNanos capture time of the current frame
     */
    fun imageMotionToRobot(ransac: StaticSceneEstimator.Result, timestampNanos: Long): RobotMotionDelta {
        if (!ransac.ok) {
            return RobotMotionDelta(0.0, 0.0, 0.0, 0.0, timestampNanos, valid = false)
        }

        val ext = calibration.extrinsics
        val fx = calibration.intrinsics.focalLengthPx
        val depthCm = ext.sceneDistanceCm

        // ---- Heading: image scene rotation -> robot yaw ----
        val dHeadingRad = ext.headingSign * ransac.thetaRad

        // ---- Translation: pixels -> cm along the camera image axes ----
        // The scene appears to move OPPOSITE to the camera:
        //   scene image motion (sxPx, syPx) = -camera motion * f / depth
        // Image axes: +x right, +y DOWN (OpenCV).
        //
        // The RANSAC fit is a similarity about the IMAGE ORIGIN, so its raw
        // (tx, ty) includes the rotation's lever arm about (0,0): pure yaw
        // would masquerade as phantom translation of ~|principalPoint| * theta
        // pixels. Re-reference the motion to the PRINCIPAL POINT — the one
        // image point where pure rotation about the optical axis produces no
        // translation — before the depth conversion.
        val cx = calibration.intrinsics.centerX
        val cy = calibration.intrinsics.centerY
        val s = ransac.scale
        val cosT = cos(ransac.thetaRad)
        val sinT = sin(ransac.thetaRad)
        val sceneMxPx = (s * (cosT * cx - sinT * cy) + ransac.txPx) - cx
        val sceneMyPx = (s * (sinT * cx + cosT * cy) + ransac.tyPx) - cy
        val cameraRightCm = -sceneMxPx * depthCm / fx
        val cameraDownCm = -sceneMyPx * depthCm / fx

        // ---- Rotate the camera-plane motion into the robot frame ----
        // imageToRobotYawDeg (MEASURE, Test 3) is the robot-frame direction
        // (CCW from +x forward) of "camera moved along image +x".
        // Image +y (down) is that direction rotated -90 deg, keeping the
        // (right, down) pair consistent in the robot frame.
        val yaw = Math.toRadians(ext.imageToRobotYawDeg)
        val dxCm = cameraRightCm * cos(yaw) + cameraDownCm * sin(yaw)
        val dyCm = cameraRightCm * sin(yaw) - cameraDownCm * cos(yaw)

        // Road Runner works in inches.
        val CM_PER_IN = 2.54
        return RobotMotionDelta(
            dxIn = dxCm / CM_PER_IN,
            dyIn = dyCm / CM_PER_IN,
            dHeadingRad = dHeadingRad,
            confidence = ransac.inlierRatio,
            timestampNanos = timestampNanos,
            valid = true,
        )
    }
}


// ============================================================================
// SECTION 3: Self-building persistent landmark map
// ============================================================================



/**
 * Lightweight per-camera "self-building" landmark map. A landmark is a spatial
 * cluster of long-lived tracks that the RANSAC repeatedly accepts as part of
 * the static scene — "persistent static visual structure", NOT a named object.
 *
 * SCOPE (honest): landmarks live in *pixel space* and persist for as long as
 * the features survive; this is enough to score scene persistence and to gate
 * confidence. Anchoring landmarks into a world-frame map (SLAM-style
 * re-association, e.g. via ORB descriptors) is future work — the
 * [OpticalFlowTracker.Track.id] bookkeeping here is the seam where that would
 * attach.
 */
class LandmarkTracker(private val cameraId: CameraId) {

    private val calibration = VisionConfig.calibrationFor(cameraId)

    private var nextLandmarkId = 0L
    private val landmarks = mutableListOf<LandmarkState>()

    /**
     * One bearing observation of a landmark, in the WORLD frame (anchored to
     * the robot's start pose): the camera position at observation time plus
     * the unit viewing-ray direction. Two or more rays from different robot
     * positions intersect at the landmark's true 3D position — this is how the
     * map gets real distances without any assumed depth.
     */
    private data class WorldObservation(val origin: DoubleArray, val dir: DoubleArray)

    private data class LandmarkState(
        val id: Long,
        var centerX: Double,
        var centerY: Double,
        val memberTrackIds: MutableSet<Long> = mutableSetOf(),
        var ageFrames: Int = 0,
        var framesSinceSeen: Int = 0,
        var observations: Int = 0,
        /** EMA of (member count * member motion consistency). */
        var stability: Double = 0.0,
        /** Bounded history of world-frame bearing rays (for triangulation). */
        val worldObservations: MutableList<WorldObservation> = mutableListOf(),
        /** Triangulated world-frame position (inches), null until mapped. */
        var worldPosIn: DoubleArray? = null,
    )

    /**
     * Recomputes landmark clusters from the tracks the tracker currently
     * considers mature AND static-scene consistent. When [robotPose] is
     * supplied, each visible landmark also accumulates a world-frame bearing
     * observation for the self-building map (triangulation needs the robot to
     * have moved a little — see WORLD_MAP_MIN_BASELINE_IN). Call once per
     * processed frame (vision thread only).
     */
    fun update(stableTracks: List<OpticalFlowTracker.Track>, robotPose: Pose2d? = null) {
        for (lm in landmarks) {
            lm.ageFrames++
            lm.framesSinceSeen++
        }

        val radiusSq = VisionTuning.LANDMARK_CLUSTER_RADIUS_PX *
            VisionTuning.LANDMARK_CLUSTER_RADIUS_PX

        // Drop memberships for tracks that disappeared.
        val aliveIds = stableTracks.map { it.id }.toHashSet()
        for (lm in landmarks) {
            lm.memberTrackIds.retainAll(aliveIds)
        }

        for (t in stableTracks) {
            // Nearest landmark within cluster radius (simple greedy clustering).
            val host = landmarks.minByOrNull {
                hypot(it.centerX - t.x, it.centerY - t.y)
            }?.takeIf {
                hypot(it.centerX - t.x, it.centerY - t.y) <=
                    VisionTuning.LANDMARK_CLUSTER_RADIUS_PX
            }

            if (host != null) {
                host.memberTrackIds.add(t.id)
                host.framesSinceSeen = 0
                host.observations++
            } else if (landmarks.size < VisionTuning.LANDMARK_MAX) {
                landmarks.add(
                    LandmarkState(
                        id = nextLandmarkId++,
                        centerX = t.x,
                        centerY = t.y,
                        memberTrackIds = mutableSetOf(t.id),
                        observations = 1,
                        stability = 0.2,
                    ),
                )
            }
        }

        // Refresh centers and stability from current member positions.
        for (lm in landmarks) {
            val members = stableTracks.filter { it.id in lm.memberTrackIds }
            if (members.isNotEmpty()) {
                lm.centerX = members.sumOf { it.x } / members.size
                lm.centerY = members.sumOf { it.y } / members.size
                val consistency = members.map { it.staticConsistency }.average()
                val target = (members.size / VisionTuning.LANDMARK_MIN_OBSERVATIONS.coerceAtLeast(1).toDouble())
                    .coerceAtMost(1.0) * consistency
                lm.stability = 0.85 * lm.stability + 0.15 * target
            } else {
                lm.stability = 0.85 * lm.stability // decays without observations
            }
        }

        // ---- Self-building world map (start-pose anchored, reference only) ----
        // Each visible landmark gets one bearing observation per frame; once
        // the robot has moved far enough between observations, the rays are
        // intersected to estimate the landmark's true world position.
        if (VisionTuning.WORLD_MAP_ENABLED && robotPose != null) {
            for (lm in landmarks) {
                if (lm.memberTrackIds.isNotEmpty()) {
                    addWorldObservation(lm, robotPose)
                }
            }
        }

        // Prune dead/empty landmarks, keep the map bounded.
        landmarks.removeAll {
            it.memberTrackIds.isEmpty() &&
                (it.framesSinceSeen > VisionTuning.LANDMARK_MIN_OBSERVATIONS || it.stability < 0.05)
        }
        while (landmarks.size > VisionTuning.LANDMARK_MAX) {
            landmarks.removeAt(landmarks.indexOf(landmarks.minByOrNull { it.stability }))
        }
    }

    /** Immutable snapshot for telemetry / debug overlay. */
    fun snapshot(timestampNanos: Long): List<VisualLandmark> =
        landmarks.map { lm ->
            VisualLandmark(
                id = lm.id,
                cameraId = cameraId,
                centerPixel = PixelPoint(lm.centerX, lm.centerY),
                featureCount = lm.memberTrackIds.size,
                ageFrames = lm.ageFrames,
                observations = lm.observations,
                stabilityScore = lm.stability,
                confidence = lm.stability *
                    if (lm.observations >= VisionTuning.LANDMARK_MIN_OBSERVATIONS) 1.0
                    else lm.observations.toDouble() / VisionTuning.LANDMARK_MIN_OBSERVATIONS,
                worldXIn = lm.worldPosIn?.get(0),
                worldYIn = lm.worldPosIn?.get(1),
                worldZIn = lm.worldPosIn?.get(2),
            )
        }

    // ---- World-map: bearing rays + triangulation ----------------------------

    /**
     * Records one world-frame bearing observation for a landmark and
     * re-triangulates its position once enough camera motion (parallax) has
     * accumulated. NOTE: the robot pose is the latest control-loop estimate,
     * not frame-synchronized, so world positions carry a small motion-scaled
     * bias (~speed x loop latency) — fine for a reference map.
     */
    private fun addWorldObservation(lm: LandmarkState, pose: Pose2d) {
        val (origin, dir) = pixelToRobotRay(lm.centerX, lm.centerY, pose)

        lm.worldObservations.add(WorldObservation(origin, dir))
        while (lm.worldObservations.size > VisionTuning.WORLD_MAP_MAX_OBSERVATIONS) {
            lm.worldObservations.removeAt(0)
        }

        if (lm.worldObservations.size < 2) return

        // Need real parallax: enough camera travel between oldest/newest rays.
        val first = lm.worldObservations.first().origin
        val last = lm.worldObservations.last().origin
        val baseline = hypot(last[0] - first[0], last[1] - first[1])
        if (baseline < VisionTuning.WORLD_MAP_MIN_BASELINE_IN) return

        triangulate(lm.worldObservations)?.let { lm.worldPosIn = it }
    }

    /**
     * Converts a pixel to a world-frame viewing ray (origin + direction) from
     * the intrinsics, the extrinsics mount angles, and the robot pose. This is
     * where the [CameraExtrinsics] yaw/pitch/roll and mount-position fields
     * become live data.
     *
     * Sign conventions (MEASURE + verify with the test procedure):
     *  - pitch: positive = optical axis tilted UP (90 = straight at ceiling).
     *  - yaw:   CCW positive, about the robot vertical axis.
     *  - roll:  applied in the image plane; flip the sign here if Test 2
     *           shows rotation inverted.
     */
    private fun pixelToRobotRay(px: Double, py: Double, pose: Pose2d): Pair<DoubleArray, DoubleArray> {
        val intr = calibration.intrinsics
        val ext = calibration.extrinsics

        // 1. Offset from the optical center, de-rolled in the image plane.
        var dx = px - intr.centerX
        var dy = py - intr.centerY
        val roll = Math.toRadians(ext.rollDeg)
        if (roll != 0.0) {
            val cr = cos(roll)
            val sr = sin(roll)
            val rx = dx * cr + dy * sr
            val ry = -dx * sr + dy * cr
            dx = rx
            dy = ry
        }

        // 2. Camera-frame direction (OpenCV pinhole: +z optical axis, +x right, +y down).
        val dCam = doubleArrayOf(dx / intr.focalLengthPx, dy / intr.focalLengthPx, 1.0)

        // 3. Mount-neutral orientation (yaw=pitch=roll=0): camera faces robot
        //    +x, image-right = robot -y, image-down = robot -z.
        var v = doubleArrayOf(dCam[2], -dCam[0], -dCam[1])

        // 4. Pitch about the robot +y (left) axis: positive lifts the axis UP.
        val p = Math.toRadians(ext.pitchDeg)
        val cp = cos(p)
        val sp = sin(p)
        v = doubleArrayOf(v[0] * cp + v[2] * sp, v[1], -v[0] * sp + v[2] * cp)

        // 5. Mount yaw about robot +z, then robot heading about world +z (CCW).
        val yaw = Math.toRadians(ext.yawDeg)
        val cy = cos(yaw)
        val sy = sin(yaw)
        v = doubleArrayOf(v[0] * cy - v[1] * sy, v[0] * sy + v[1] * cy, v[2])
        val h = pose.heading.toDouble()
        val ch = cos(h)
        val sh = sin(h)
        val dir = doubleArrayOf(v[0] * ch - v[1] * sh, v[0] * sh + v[1] * ch, v[2])

        // 6. Ray origin: robot position + rotated camera mount offset (cm -> in).
        val cmToIn = 1.0 / 2.54
        val ox = ext.positionXcm * cmToIn
        val oy = ext.positionYcm * cmToIn
        val origin = doubleArrayOf(
            pose.position.x + ox * ch - oy * sh,
            pose.position.y + ox * sh + oy * ch,
            ext.positionZcm * cmToIn,
        )
        return origin to dir
    }

    /**
     * Least-squares intersection of the bearing rays: minimizes
     * sum_i |X - (o_i + t_i d_i)|^2, which reduces to the 3x3 system
     * A X = b with A = sum_i (I - d_i d_i^T), b = sum_i (I - d_i d_i^T) o_i.
     * Returns null when the rays are nearly parallel (ill-conditioned) or the
     * solution is non-finite/absurd.
     */
    private fun triangulate(obs: List<WorldObservation>): DoubleArray? {
        val a = Array(3) { DoubleArray(3) }
        val b = DoubleArray(3)
        for (o in obs) {
            val d = o.dir
            for (r in 0..2) {
                for (c in 0..2) {
                    val proj = (if (r == c) 1.0 else 0.0) - d[r] * d[c]
                    a[r][c] += proj
                    b[r] += proj * o.origin[c]
                }
            }
        }

        val det = det3(a)
        if (abs(det) < 1e-9) return null // rays nearly parallel: no parallax info

        val x = DoubleArray(3)
        for (i in 0..2) {
            x[i] = det3(replaceColumn(a, i, b)) / det
        }
        if (x.any { !it.isFinite() || abs(it) > 10_000.0 }) return null
        return x
    }

    private fun det3(m: Array<DoubleArray>): Double =
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
            m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
            m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])

    private fun replaceColumn(m: Array<DoubleArray>, col: Int, v: DoubleArray): Array<DoubleArray> =
        Array(3) { r ->
            DoubleArray(3) { c -> if (c == col) v[r] else m[r][c] }
        }

    /** Number of landmarks currently considered trustworthy scene anchors. */
    fun matureCount(): Int = landmarks.count {
        it.observations >= VisionTuning.LANDMARK_MIN_OBSERVATIONS && it.stability > 0.4
    }
}
