package org.firstinspires.ftc.teamcode.vision

import org.opencv.core.Mat
import org.opencv.core.MatOfByte
import org.opencv.core.MatOfFloat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.core.Size
import org.opencv.core.TermCriteria
import org.opencv.imgproc.Imgproc
import org.opencv.video.Video

// ============================================================================
// SECTION 1: Corner detection abstraction + Shi-Tomasi detector
// ============================================================================



/**
 * Abstraction over corner detection so Shi-Tomasi can be swapped for FAST,
 * ORB, etc. later without touching the tracker. Implementations must be
 * cheap enough for a 640x480 frame on FTC control-hub hardware.
 */
interface FeatureDetector {
    /**
     * Detects up to [maxCount] corner features in a grayscale image.
     * @param gray grayscale frame
     * @param existing points to avoid when placing new corners (may be empty)
     * @return detected pixel coordinates (MatOfPoint2f, owned by the caller)
     */
    fun detect(gray: Mat, existing: List<Point>, maxCount: Int): MatOfPoint2f
}

/**
 * Shi-Tomasi "good features to track" — lightweight, well-proven for
 * Lucas-Kanade tracking, and already bundled with the FTC SDK's OpenCV.
 */
class ShiTomasiDetector : FeatureDetector {

    override fun detect(gray: Mat, existing: List<Point>, maxCount: Int): MatOfPoint2f {
        if (maxCount <= 0) return MatOfPoint2f()

        val candidates = MatOfPoint()
        Imgproc.goodFeaturesToTrack(
            gray,
            candidates,
            maxCount,
            VisionTuning.DETECTION_QUALITY,
            VisionTuning.DETECTION_MIN_DISTANCE_PX,
        )

        val raw = candidates.toArray()
        candidates.release()

        if (existing.isEmpty()) {
            val out = MatOfPoint2f()
            out.fromList(raw.toList())
            return out
        }

        // Keep only points far enough from already-tracked corners so we do
        // not duplicate tracks. The detection minDistance only spaces the new
        // candidates from each other, not from old tracks.
        val minDistSq = VisionTuning.DETECTION_MIN_DISTANCE_PX *
            VisionTuning.DETECTION_MIN_DISTANCE_PX
        val kept = raw.filter { p ->
            existing.none { o ->
                val dx = p.x - o.x
                val dy = p.y - o.y
                dx * dx + dy * dy < minDistSq
            }
        }
        val filtered = MatOfPoint2f()
        filtered.fromList(kept)
        return filtered
    }
}

// ============================================================================
// SECTION 2: Pyramidal Lucas-Kanade feature tracking with track lifecycle
// ============================================================================



/**
 * Per-camera feature tracker: pyramidal Lucas-Kanade optical flow with
 * track lifecycle (age, success count, error), periodic reacquisition, and
 * RANSAC-consistency bookkeeping. Runs entirely on the vision thread.
 */
class OpticalFlowTracker(
    val cameraId: CameraId,
    private val detector: FeatureDetector = ShiTomasiDetector(),
) {
    /** Mutable track state; only ever touched by the vision thread. */
    data class Track(
        val id: Long,
        var x: Double,
        var y: Double,
        var prevX: Double,
        var prevY: Double,
        var ageFrames: Int = 0,
        var successfulFrames: Int = 0,
        var lastErrorPx: Double = 0.0,
        /** EMA of per-frame LK error. */
        var emaErrorPx: Double = 0.0,
        /** EMA of RANSAC agreement (1 when always an inlier). */
        var staticConsistency: Double = 0.5,
        /** Set for one frame so the UI can color new tracks. */
        var isNew: Boolean = false,
        /** True when the most recent RANSAC classified this track as static-scene. */
        var isStaticInlier: Boolean = false,
    )

    private var nextId = 0L
    private val tracks = mutableListOf<Track>()
    private var prevGray: Mat? = null
    private var framesSinceDetect = Int.MAX_VALUE

    /**
     * Compact per-frame tracking result handed to RANSAC and the snapshot
     * builder. [prevXy]/[curXy] contain ONLY tracks that were actually LK-
     * tracked this frame (freshly reacquired tracks are excluded — their
     * prev==cur pair is an artifact, not a measurement, and a pile of
     * zero-motion correspondences biases the RANSAC consensus toward "no
     * motion"). [trackIndices] maps each correspondence-array index to its
     * index in the tracker's track list so inlier classifications can be
     * applied to the right [Track]s.
     */
    data class FrameResult(
        val prevXy: DoubleArray,
        val curXy: DoubleArray,
        val trackedCount: Int,
        val meanErrorPx: Double,
        val detectionsThisFrame: Int,
        val trackIndices: IntArray = IntArray(0),
    )

    /**
     * Advances tracking with a new grayscale frame.
     * @return correspondences for RANSAC (prev/cur interleaved x,y) plus stats
     */
    fun processFrame(gray: Mat): FrameResult {
        val prev = prevGray
        if (prev == null || prev.size() != gray.size() || tracks.isEmpty()) {
            reacquire(gray, fullReplace = prev == null || prev.size() != gray.size())
            storeGray(gray)
            return FrameResult(DoubleArray(0), DoubleArray(0), 0, 0.0, tracks.size)
        }

        // ---- 1. Track existing features with pyramidal LK ----
        val prevPts = MatOfPoint2f()
        prevPts.fromList(tracks.map { Point(it.x, it.y) })
        val nextPts = MatOfPoint2f()
        val status = MatOfByte()
        val err = MatOfFloat()

        Video.calcOpticalFlowPyrLK(
            prev, gray, prevPts, nextPts, status, err,
            Size(VisionTuning.LK_WINDOW_SIZE.toDouble(), VisionTuning.LK_WINDOW_SIZE.toDouble()),
            VisionTuning.LK_PYRAMID_LEVELS,
            TermCriteria(TermCriteria.COUNT + TermCriteria.EPS, 30, 0.01),
            0, 1e-4,
        )

        val nextArr = nextPts.toArray()
        val statusArr = status.toArray()
        val errArr = err.toArray()

        var errorSum = 0.0
        var errorCount = 0
        val survivors = mutableListOf<Track>()
        for (i in tracks.indices) {
            val t = tracks[i]
            if (i >= nextArr.size || statusArr.getOrNull(i)?.toInt() != 1) continue

            val nx = nextArr[i].x
            val ny = nextArr[i].y
            val e = errArr.getOrNull(i)?.toDouble() ?: 0.0
            if (e > VisionTuning.LK_MAX_ERROR_PX || !nx.isFinite() || !ny.isFinite()) continue

            t.prevX = t.x
            t.prevY = t.y
            t.x = nx
            t.y = ny
            t.ageFrames++
            t.successfulFrames++
            t.lastErrorPx = e
            t.emaErrorPx = if (t.emaErrorPx == 0.0) e else 0.8 * t.emaErrorPx + 0.2 * e
            t.isNew = false
            errorSum += t.emaErrorPx
            errorCount++
            survivors.add(t)
        }

        prevPts.release(); nextPts.release(); status.release(); err.release()

        // Keep survivors; retire stale ones unless they are proven static-scene anchors
        tracks.clear()
        for (t in survivors) {
            if (t.ageFrames < VisionTuning.LK_MAX_FEATURE_AGE_FRAMES || t.isStaticInlier) {
                tracks.add(t)
            }
        }

        // ---- 2. Periodic reacquisition of fresh features ----
        var detections = 0
        framesSinceDetect++
        if (tracks.size < VisionTuning.LK_MIN_TRACKS ||
            framesSinceDetect >= VisionTuning.LK_REACQUIRE_INTERVAL_FRAMES
        ) {
            detections = reacquire(gray, fullReplace = false)
        }

        // ---- 3. Build correspondence arrays for RANSAC ----
        // Skip isNew tracks: they were reacquired this frame and carry no
        // prev->cur measurement (prev == cur). Feeding them in would add
        // zero-motion "votes" that a no-motion RANSAC hypothesis trivially
        // accepts, skewing the scene-motion estimate toward zero.
        val n = tracks.size // total live tracks (confidence uses this)
        val prevXy = DoubleArray(tracks.size * 2)
        val curXy = DoubleArray(tracks.size * 2)
        val trackIndices = IntArray(tracks.size)
        var correspondenceCount = 0
        for (i in tracks.indices) {
            val t = tracks[i]
            if (t.isNew) continue
            val out = correspondenceCount * 2
            trackIndices[correspondenceCount] = i
            prevXy[out] = t.prevX
            prevXy[out + 1] = t.prevY
            curXy[out] = t.x
            curXy[out + 1] = t.y
            correspondenceCount++
        }

        storeGray(gray)

        return FrameResult(
            prevXy.copyOf(correspondenceCount * 2),
            curXy.copyOf(correspondenceCount * 2),
            trackedCount = n,
            meanErrorPx = if (errorCount > 0) errorSum / errorCount else 0.0,
            detectionsThisFrame = detections,
            trackIndices = trackIndices.copyOf(correspondenceCount),
        )
    }

    /**
     * Marks which tracks the RANSAC considered part of the static scene.
     * [inlierIndices] index the CORRESPONDENCE arrays; [trackIndices] (from
     * the same [FrameResult]) map them back onto track-list positions.
     */
    fun applyRansacResult(inlierIndices: IntArray, trackIndices: IntArray) {
        val inliers = BooleanArray(tracks.size)
        for (k in inlierIndices) {
            val trackIndex = trackIndices.getOrNull(k) ?: continue
            if (trackIndex in inliers.indices) inliers[trackIndex] = true
        }
        for (i in tracks.indices) {
            val t = tracks[i]
            t.isStaticInlier = inliers[i]
            val reward = if (t.isStaticInlier) 1.0 else 0.0
            t.staticConsistency = 0.9 * t.staticConsistency + 0.1 * reward
        }
    }

    /** Current track snapshot (immutable), for the debug overlay + telemetry. */
    fun snapshotFeatures(timestampNanos: Long): List<TrackedFeature> =
        tracks.map { t ->
            TrackedFeature(
                id = t.id,
                cameraId = cameraId,
                previousPixel = PixelPoint(t.prevX, t.prevY),
                currentPixel = PixelPoint(t.x, t.y),
                ageFrames = t.ageFrames,
                successfulFrames = t.successfulFrames,
                trackingErrorPx = t.emaErrorPx,
                staticConsistency = t.staticConsistency,
                isStaticInlier = t.isStaticInlier,
                isNew = t.isNew,
                lastSeenTimestampNanos = timestampNanos,
            )
        }

    /** Long-lived tracks the RANSAC currently accepts as static-scene members. */
    fun matureStaticTracks(): List<Track> = tracks.filter {
        it.successfulFrames >= VisionTuning.LK_MIN_SUCCESS_FRAMES && it.isStaticInlier
    }

    /** True when this track is a mature, repeatedly-accepted static member. */
    fun isStable(t: Track): Boolean =
        t.ageFrames >= VisionTuning.LANDMARK_MIN_AGE_FRAMES &&
            t.staticConsistency >= 0.6

    fun reset() {
        tracks.clear()
        framesSinceDetect = Int.MAX_VALUE
        prevGray?.release()
        prevGray = null
    }

    private fun storeGray(gray: Mat) {
        val previous = prevGray
        if (previous == null || previous.size() != gray.size() || previous.type() != gray.type()) {
            prevGray?.release()
            prevGray = gray.clone()
        } else {
            gray.copyTo(previous)
        }
    }

    /** Detects new corners, merging them into the track list. */
    private fun reacquire(gray: Mat, fullReplace: Boolean): Int {
        if (fullReplace) tracks.clear()

        val available = VisionTuning.MAX_FEATURES - tracks.size
        if (available <= 0) {
            framesSinceDetect = 0
            return 0
        }

        val existing = tracks.map { Point(it.x, it.y) }
        val fresh = detector.detect(gray, existing, available)
        val points = fresh.toArray()
        for (p in points) {
            tracks.add(Track(id = nextId++, x = p.x, y = p.y, prevX = p.x, prevY = p.y, isNew = true))
        }
        fresh.release()
        framesSinceDetect = 0
        return points.size
    }

}
