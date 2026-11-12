package org.firstinspires.ftc.teamcode.vision

import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import com.acmerobotics.roadrunner.Pose2d
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc
import java.util.concurrent.atomic.AtomicReference
import kotlin.math.min

/**
 * FTC SDK VisionProcessor running the full per-camera pipeline on the VISION
 * thread (VisionPortal worker), so the NextFTC/OpMode loop is never blocked by
 * image processing:
 *
 *   camera frame -> Shi-Tomasi detection -> pyramidal LK tracking
 *     -> RANSAC static-scene fit -> visual motion estimate
 *     -> immutable [CameraObservation] snapshot (AtomicReference)
 *
 * The control loop always consumes the latest complete snapshot and rejects
 * stale ones by timestamp. Both cameras run one of these, feeding ONE
 * [HybridPoseEstimator]; a blocked/failed camera simply stops publishing.
 */
class VisionCameraProcessor(private val cameraId: CameraId) : VisionProcessor {

    private val calibration = VisionConfig.calibrationFor(cameraId)
    private val tracker = OpticalFlowTracker(cameraId)
    private val sceneEstimator = StaticSceneEstimator()
    private val motionEstimator = VisualMotionEstimator(calibration)
    private val landmarkTracker = LandmarkTracker(cameraId)

    private val latest = AtomicReference<CameraObservation?>(null)

    /** Latest robot pose pushed by the control loop (for the world landmark map). */
    private val robotPoseRef = AtomicReference<Pose2d?>(null)

    /**
     * Called by [VisionSubsystem] each control-loop iteration with the best
     * available pose estimate. Thread-safe; consumed by the vision thread when
     * recording world-map observations.
     */
    fun updateRobotPose(pose: Pose2d?) {
        robotPoseRef.set(pose)
    }

    /** Vision-thread-only counters (processFrame is single-threaded per portal). */
    private var frameCounter = 0
    private var lastProcessNanos = 0L
    private var fpsEma = 0.0
    private var confidenceEma = 0.0

    /** Debug drawing state (created lazily on the UI thread). */
    private var paintFill: Paint? = null
    private var paintText: Paint? = null

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
        tracker.reset()
        lastProcessNanos = 0L
        fpsEma = 0.0
        confidenceEma = 0.0
    }

    override fun processFrame(frame: Mat?, captureTimeNanos: Long): Any? {
        val img = frame ?: return null

        // Adaptive frame decimation: vary processing rate based on confidence
        frameCounter++
        val decimation = if (VisionTuning.ADAPTIVE_FRAME_DECIMATION && confidenceEma > 0.0) {
            when {
                confidenceEma >= VisionTuning.ADAPTIVE_HIGH_CONF_THRESHOLD -> VisionTuning.ADAPTIVE_DECIMATION_HIGH_CONF
                confidenceEma <= VisionTuning.ADAPTIVE_LOW_CONF_THRESHOLD -> VisionTuning.ADAPTIVE_DECIMATION_LOW_CONF
                else -> VisionTuning.ADAPTIVE_DECIMATION_MED_CONF
            }
        } else {
            VisionTuning.PROCESS_EVERY_NTH_FRAME
        }
        if (decimation > 1 && frameCounter % decimation != 0) {
            return latest.get()
        }

        // ---- FPS estimate over processed frames ----
        if (lastProcessNanos != 0L && captureTimeNanos > lastProcessNanos) {
            val dtSec = (captureTimeNanos - lastProcessNanos) / 1e9
            val instant = 1.0 / dtSec
            fpsEma = if (fpsEma == 0.0) instant else 0.9 * fpsEma + 0.1 * instant
        }
        lastProcessNanos = captureTimeNanos

        val gray = Mat()
        Imgproc.cvtColor(img, gray, Imgproc.COLOR_RGB2GRAY)

        // ---- Track features ----
        val result = tracker.processFrame(gray)

        // ---- Robust static-scene fit ----
        val ransac = if (result.trackedCount >= VisionTuning.RANSAC_MIN_INLIERS) {
            sceneEstimator.estimate(result.prevXy, result.curXy)
        } else {
            StaticSceneEstimator.Result(
                ok = false, thetaRad = 0.0, txPx = 0.0, tyPx = 0.0, scale = 1.0,
                inlierIndices = IntArray(0), inlierRatio = 0.0,
            )
        }
        // inlier indices refer to the correspondence arrays — pass the map
        // back to the tracker so classifications land on the right tracks.
        tracker.applyRansacResult(ransac.inlierIndices, result.trackIndices)

        // ---- Robot-frame relative motion ----
        val motion = motionEstimator.imageMotionToRobot(ransac, captureTimeNanos)

        // ---- Persistent landmark map (incl. world triangulation) ----
        landmarkTracker.update(tracker.matureStaticTracks(), robotPoseRef.get())

        // ---- Per-camera confidence ----
        val rawConfidence = computeRawConfidence(result, ransac)
        confidenceEma = confidenceEma +
            VisionTuning.CONF_EMA_ALPHA * (rawConfidence - confidenceEma)
        confidenceEma = confidenceEma.coerceIn(0.0, 1.0)

        // ---- Publish an immutable snapshot ----
        val observation = CameraObservation(
            cameraId = cameraId,
            timestampNanos = captureTimeNanos,
            fps = fpsEma,
            detectedCount = result.detectionsThisFrame + result.trackedCount,
            trackedCount = result.trackedCount,
            inlierCount = ransac.inlierIndices.size,
            outlierCount = result.trackedCount - ransac.inlierIndices.size,
            ransacOk = ransac.ok,
            inlierRatio = ransac.inlierRatio,
            meanTrackErrorPx = result.meanErrorPx,
            confidence = confidenceEma,
            motion = motion,
            features = tracker.snapshotFeatures(captureTimeNanos),
            landmarks = landmarkTracker.snapshot(captureTimeNanos),
        )
        latest.set(observation)

        gray.release()
        return observation
    }

    private fun computeRawConfidence(
        result: OpticalFlowTracker.FrameResult,
        ransac: StaticSceneEstimator.Result,
    ): Double {
        if (!ransac.ok || result.trackedCount < VisionTuning.RANSAC_MIN_INLIERS) return 0.0

        val trackFactor = min(
            1.0,
            result.trackedCount / VisionTuning.CONF_MIN_TRACKS_FOR_FULL,
        )
        val inlierFactor = ransac.inlierRatio.coerceIn(0.0, 1.0)
        val errorFactor =
            (1.0 - result.meanErrorPx / VisionTuning.LK_MAX_ERROR_PX).coerceIn(0.0, 1.0)

        return trackFactor * (0.6 * inlierFactor + 0.4 * errorFactor)
    }

    /** Latest immutable snapshot (thread-safe; null before the first frame). */
    fun latestObservation(): CameraObservation? = latest.get()

    fun reset() {
        tracker.reset()
        latest.set(null)
        frameCounter = 0
        lastProcessNanos = 0L
        fpsEma = 0.0
        confidenceEma = 0.0
    }

    /**
     * Debug visualization (enable via VisionTuning.DEBUG_OVERLAY):
     *   GREEN  = accepted static-scene feature (RANSAC inlier)
     *   RED    = rejected / outlier feature (likely moving object or bad track)
     *   BLUE   = newly detected feature (not yet classified)
     *   YELLOW = persistent landmark cluster center
     *   WHITE  = dominant scene motion vector + status text
     *
     * [userContext] is the immutable [CameraObservation] returned by
     * [processFrame], so drawing is race-free by construction.
     */
    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?,
    ) {
        if (!VisionTuning.DEBUG_OVERLAY) return
        val c = canvas ?: return
        val obs = userContext as? CameraObservation ?: return

        val fill = paintFill ?: Paint().apply {
            style = Paint.Style.FILL
            isAntiAlias = true
        }.also { paintFill = it }
        val text = paintText ?: Paint().apply {
            color = Color.WHITE
            textSize = 22f
            isAntiAlias = true
            setShadowLayer(3f, 1f, 1f, Color.BLACK)
        }.also { paintText = it }

        val s = scaleBmpPxToCanvasPx

        // Features with motion vectors
        for (f in obs.features) {
            val x = (f.currentPixel.x * s).toFloat()
            val y = (f.currentPixel.y * s).toFloat()
            val px = (f.previousPixel.x * s).toFloat()
            val py = (f.previousPixel.y * s).toFloat()

            fill.color = when {
                f.isNew -> Color.BLUE
                f.isStaticInlier -> Color.GREEN
                else -> Color.RED
            }
            c.drawCircle(x, y, 4f, fill)
            c.drawLine(px, py, x, y, fill)
        }

        // Landmark cluster centers
        fill.color = Color.YELLOW
        for (lm in obs.landmarks) {
            val x = (lm.centerPixel.x * s).toFloat()
            val y = (lm.centerPixel.y * s).toFloat()
            c.drawCircle(x, y, 10f, fill)
            c.drawCircle(x, y, 16f, fill)
        }

        // World-mapped landmarks (triangulated start-pose coordinates) get a
        // white center marker so you can see the map forming on the DS view.
        fill.color = Color.WHITE
        for (lm in obs.landmarks) {
            if (lm.worldXIn != null) {
                val x = (lm.centerPixel.x * s).toFloat()
                val y = (lm.centerPixel.y * s).toFloat()
                c.drawCircle(x, y, 5f, fill)
            }
        }

        // Status text
        text.color = Color.WHITE
        c.drawText(
            "${cameraId.name}  conf=${"%.2f".format(obs.confidence)}  " +
                "trk=${obs.trackedCount} in=${obs.inlierCount} out=${obs.outlierCount}  " +
                "mapped=${obs.landmarks.count { it.worldXIn != null }}  " +
                "fps=${"%.0f".format(obs.fps)}",
            12f, 30f, text,
        )
        if (obs.motion != null && obs.motion.valid) {
            val m = obs.motion
            text.color = if (obs.ransacOk) Color.GREEN else Color.RED
            c.drawText(
                "vis dx=${"%.2f".format(m.dxIn)}in dy=${"%.2f".format(m.dyIn)}in " +
                    "dh=${"%.1f".format(Math.toDegrees(m.dHeadingRad))}deg",
                12f, 58f, text,
            )
        }
    }
}

