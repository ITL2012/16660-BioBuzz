package org.firstinspires.ftc.teamcode

/**
 * ============ NEURAL NETS (all on-robot) ============
 *
 * One merged home for the neural stack (was the nn/ package):
 *  * [ShotPredictor]     — tiny pure-Kotlin MLP: P(shot scores); ADVISORY
 *                          logs it, REFEREE nudges RPM (bounded +/-5%)
 *  * [NeuralReferee]     — the authority ladder the BallLocator consults
 *  * [DetectorProcessor] — tiny YOLO via TFLite on the FRONT camera
 *
 * All inference runs inside the Robot Controller app; models live in the
 * app's external storage (nn/ folder, next to blackbox/). Train offline with
 * the tools/ scripts — see tools/README.md.
 */

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.json.JSONArray
import org.json.JSONObject
import org.opencv.core.Mat
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc
import org.tensorflow.lite.Interpreter
import java.io.File
import java.io.FileInputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.MappedByteBuffer
import java.nio.channels.FileChannel
import kotlin.math.atan2
import kotlin.math.exp
import kotlin.math.max
import kotlin.math.tanh


/**
 * ============ SHOT-OUTCOME PREDICTOR — a tiny MLP in pure Kotlin ============
 *
 * Neural net #1 of the staged plan (B): learns, from every logged shot, the
 * probability that a ball fired at (distance, RPM, battery voltage) actually
 * scores. Trained OFFLINE by tools/train_shot_predictor.py on your laptop
 * (numpy only), exported as a small JSON of weights, copied to the RC app's
 * storage at [SHOT_PREDICTOR_FILE], and run here with a few dozen multiply
 * adds — no TFLite, no latency, nothing to install on the robot.
 *
 * Authority ([SHOT_PREDICTOR_MODE], default ADVISORY):
 *  * ADVISORY — logs P(hit) to the Black Box + telemetry only. Run this for
 *    several matches to prove the model before flipping.
 *  * REFEREE  — when P(hit) is below SHOT_PREDICTOR_MIN_CONFIDENCE, nudges
 *    the commanded RPM in the direction the model's gradient says improves
 *    the odds, bounded to +/- SHOT_PREDICTOR_MAX_ADJUST (5%).
 */
object ShotPredictor {

    @Volatile
    var available = false
        private set

    private var loadedOnce = false
    private var w1: Array<DoubleArray> = emptyArray() // [hidden][3]
    private var b1 = DoubleArray(0)
    private var w2 = DoubleArray(0)                    // [hidden]
    private var b2 = 0.0
    private var mean = DoubleArray(3)
    private var std = DoubleArray(3)
    private var lastLogNanos = 0L

    /** Loads the model once per app run. Safe to call from every OpMode. */
    fun ensureLoaded(context: Context) {
        if (loadedOnce) return
        loadedOnce = true
        if (SHOT_PREDICTOR_MODE == NnMode.OFF) return
        try {
            val f = File(context.getExternalFilesDir(null), SHOT_PREDICTOR_FILE)
            if (!f.exists()) {
                BlackBox.event("nn_status", "component" to "shot_predictor", "status" to "no model file")
                return
            }
            val root = JSONObject(f.readText())
            mean = doubles(root.getJSONArray("mean"))
            std = doubles(root.getJSONArray("std"))
            val ja = root.getJSONArray("w1")
            w1 = Array(ja.length()) { doubles(ja.getJSONArray(it)) }
            b1 = doubles(root.getJSONArray("b1"))
            w2 = doubles(root.getJSONArray("w2"))
            b2 = root.getDouble("b2")
            available = true
            BlackBox.event(
                "nn_status", "component" to "shot_predictor",
                "status" to "loaded", "hidden" to w1.size,
            )
        } catch (e: Exception) {
            BlackBox.event(
                "nn_status", "component" to "shot_predictor",
                "status" to "error", "msg" to (e.message ?: "?"),
            )
        }
    }

    private fun doubles(ja: JSONArray): DoubleArray =
        DoubleArray(ja.length()) { ja.getDouble(it) }

    /**
     * P(this shot scores) for the given shot parameters, from the trained
     * model. Null when no model is loaded. Features are normalized with the
     * training set's mean/std before the forward pass.
     */
    fun pHit(distanceCm: Double, rpm: Double, batteryVolts: Double): Double? {
        if (!available) return null
        val x = doubleArrayOf(
            (distanceCm - mean[0]) / std[0],
            (rpm - mean[1]) / std[1],
            (batteryVolts - mean[2]) / std[2],
        )
        val h = DoubleArray(w1.size)
        for (i in w1.indices) {
            var z = b1[i]
            val row = w1[i]
            for (j in 0 until 3) z += row[j] * x[j]
            h[i] = tanh(z)
        }
        var z2 = b2
        for (i in h.indices) z2 += w2[i] * h[i]
        return 1.0 / (1.0 + exp(-z2))
    }

    /**
     * Latest battery voltage, in volts. The OpModes feed this from their
     * hardwareMap voltage sensor (they already read it for the Black Box);
     * defaults to a healthy 12V if nobody has reported yet.
     */
    @Volatile
    var lastKnownBatteryVolts = 12.0
        private set

    /** Reads + stores the hub battery voltage; returns it (0.0 if unreadable). */
    fun reportHardwareMapVolts(hardwareMap: HardwareMap): Double {
        val v = try {
            hardwareMap.voltageSensor.iterator().next().voltage
        } catch (_: Exception) {
            0.0
        }
        if (v > 0) lastKnownBatteryVolts = v
        return v
    }

    fun batteryVolts(): Double = lastKnownBatteryVolts

    /**
     * The opinionated API the fire paths call: returns the RPM to actually
     * command. ADVISORY logs P(hit) (throttled to ~1 Hz); REFEREE also nudges
     * the RPM when the model doesn't like the shot, bounded to
     * [SHOT_PREDICTOR_MAX_ADJUST]. Never changes anything with no model, no
     * distance, or a good-looking shot.
     */
    fun adjustRpm(rpm: Double, distanceCm: Double, source: String): Double {
        if (!available || distanceCm <= 0.0) return rpm
        val p = pHit(distanceCm, rpm, batteryVolts()) ?: return rpm

        // throttled advisory logging (this is called every loop tick)
        val now = System.nanoTime()
        if (now - lastLogNanos > 1_000_000_000L) {
            lastLogNanos = now
            BlackBox.event(
                "nn_shot",
                "p" to p, "rpm" to rpm, "distanceCm" to distanceCm,
                "batteryV" to batteryVolts(), "source" to source,
            )
        }

        if (SHOT_PREDICTOR_MODE != NnMode.REFEREE) return rpm
        if (p >= SHOT_PREDICTOR_MIN_CONFIDENCE) return rpm

        // direction the model says improves the odds (numeric gradient wrt rpm)
        val eps = rpm * 0.01
        val pUp = pHit(distanceCm, rpm + eps, batteryVolts()) ?: p
        val grad = (pUp - p) / eps
        if (grad == 0.0) return rpm
        val dir = if (grad > 0) 1.0 else -1.0
        val adjusted = rpm * (1.0 + dir * SHOT_PREDICTOR_MAX_ADJUST)
        BlackBox.event(
            "nn_shot_adjust",
            "p" to p, "rpmIn" to rpm, "rpmOut" to adjusted,
            "distanceCm" to distanceCm, "source" to source,
        )
        return adjusted
    }
}


/**
 * ============ NEURAL REFEREE — the authority ladder's enforcement point ============
 *
 * Holds the detector's latest verdicts (published by [DetectorProcessor] at
 * NN_DETECT_HZ) and applies them to the classical HSV pipeline's targets
 * according to [NN_DETECTOR_MODE]:
 *
 *  * OFF / ADVISORY — HSV targets pass through untouched (advisory verdicts
 *    are visible on the DS preview + in the Black Box only).
 *  * REFEREE — while a FRESH verdict exists, any HSV blob the detector does
 *    NOT corroborate is removed (kills lighting hallucinations). If the
 *    verdict is stale, HSV is trusted as-is — the referee never acts blind.
 *  * PRIMARY — REFEREE behavior PLUS detector boxes no HSV blob matched are
 *    added as brand-new targets (the NN can truly drive the game).
 */
object NeuralReferee {

    data class Detection(
        val labelId: Int,
        val confidence: Double,
        /** Full-frame pixel box: left, top, width, height. */
        val x: Double,
        val y: Double,
        val w: Double,
        val h: Double,
    ) {
        val label: String get() = NN_LABELS.getOrElse(labelId) { "?" }
        val centerX: Double get() = x + w / 2
        val centerY: Double get() = y + h / 2
    }

    @Volatile
    private var latestList: List<Detection> = emptyList()

    @Volatile
    private var latestNanos = 0L

    /** Called by the DetectorProcessor after each inference (throttled inside). */
    fun publish(dets: List<Detection>, captureNanos: Long) {
        latestList = dets
        latestNanos = captureNanos
        // One event per detection: the rows are typed, so the training
        // tools can filter on class and confidence with plain SQL.
        for (d in dets) {
            BlackBox.event(
                "nn_det",
                "cls" to d.label, "conf" to d.confidence,
                "x" to d.x, "y" to d.y, "w" to d.w, "h" to d.h,
            )
        }
    }

    fun latest(): List<Detection> = latestList

    /** True when the detector has spoken recently enough to referee. */
    fun fresh(): Boolean =
        System.nanoTime() - latestNanos < (NN_REF_STALE_MS * 1e6).toLong()

    /**
     * Applies the authority ladder to the HSV pipeline's targets. Called from
     * BallLocator.visibleTargets(). In PRIMARY mode, a sustained ghost rate
     * (detector-only additions the HSV pipeline never sees) auto-demotes to
     * REFEREE for the rest of the OpMode — an untrained or regressed model
     * can hallucinate balls, and the pursuit would faithfully chase them.
     */
    fun filter(targets: List<BallTarget>): List<BallTarget> {
        if (NN_DETECTOR_MODE == NnMode.OFF || NN_DETECTOR_MODE == NnMode.ADVISORY) {
            return targets
        }
        if (!fresh() || latestList.isEmpty()) {
            return targets // referee is asleep -> trust the classical pipeline
        }

        val kept = targets.filter { corroborated(it) }

        // Agreement statistic: how many HSV blobs the referee removed
        // (a HIGH rate means either HSV hallucinates or the NN is blind).
        val a = NN_AGREE_EMA_ALPHA
        hsvKilledEma += a * (
            (targets.size - kept.size).toDouble() / targets.size.coerceAtLeast(1) - hsvKilledEma
            )

        if (NN_DETECTOR_MODE == NnMode.REFEREE || demoted) {
            return kept
        }

        // PRIMARY: also add detector boxes the HSV pipeline never matched.
        val additions = latestList
            .filter { d -> d.labelId in NN_LABELS.indices && d.label != "?" }
            .filter { d -> targets.none { t -> sameLabel(d, t) && insideBox(t.screenX, t.screenY, d) } }
            .map { d -> d.toBallTarget() }

        // Ghost statistic: detector-only additions / total NN detections.
        ghostEma += a * (additions.size.toDouble() / latestList.size.coerceAtLeast(1) - ghostEma)
        if (!demoted && ghostEma > NN_GHOST_RATE_LIMIT) {
            demoted = true
            BlackBox.event(
                "nn_demoted",
                "ghostRate" to ghostEma,
                "limit" to NN_GHOST_RATE_LIMIT,
                "modelVersion" to NN_MODEL_VERSION,
            )
            return kept // behave as REFEREE from here on
        }
        return kept + additions
    }

    // ---- agreement / ghost statistics (PRIMARY safety + telemetry) ----

    /** EMA of detector-only additions as a fraction of all NN detections. */
    private var ghostEma = 0.0

    /** EMA of HSV blobs the referee removed as a fraction of all HSV blobs. */
    private var hsvKilledEma = 0.0

    /** PRIMARY -> REFEREE auto-demotion latch (per OpMode run). */
    private var demoted = false

    fun ghostRate(): Double = ghostEma

    fun hsvKilledRate(): Double = hsvKilledEma

    fun autoDemoted(): Boolean = demoted

    fun modeName(): String =
        if (demoted) "PRIMARY->REFEREE (auto-demoted)" else NN_DETECTOR_MODE.name

    /** Live agreement stats for the Driver Station. */
    fun addTelemetry(t: org.firstinspires.ftc.robotcore.external.Telemetry) {
        t.addData("NN mode", modeName())
        t.addData(
            "NN referee",
            "ghost %.0f%% | HSV-killed %.0f%% | %s".format(
                ghostEma * 100, hsvKilledEma * 100, if (fresh()) "fresh" else "asleep",
            ),
        )
    }

    /** Resets the per-run statistics (called at OpMode start). */
    fun resetStats() {
        ghostEma = 0.0
        hsvKilledEma = 0.0
        demoted = false
    }

    private fun corroborated(t: BallTarget): Boolean =
        latestList.any { d -> sameLabel(d, t) && insideBox(t.screenX, t.screenY, d) }

    private fun sameLabel(d: Detection, t: BallTarget): Boolean = d.label == t.label

    private fun insideBox(px: Double, py: Double, d: Detection): Boolean {
        val tx = d.w * (NN_REF_MARGIN - 1.0) / 2
        val ty = d.h * (NN_REF_MARGIN - 1.0) / 2
        return px > d.x - tx && px < d.x + d.w + tx &&
            py > d.y - ty && py < d.y + d.h + ty
    }

    /** Same pinhole math as BallLocator, for detector-added PRIMARY targets. */
    private fun Detection.toBallTarget(): BallTarget {
        val pixelWidth = max(w, h).coerceAtLeast(2.0)
        val widthMm = when (label) {
            "Pollen" -> POLLEN_PHYSICAL_WIDTH_MM
            "Flower" -> FLOWER_PHYSICAL_WIDTH_MM
            else -> NECTAR_PHYSICAL_WIDTH_MM
        }
        return BallTarget(
            label = label,
            distanceCm = C270_FOCAL_LENGTH_PIXELS * widthMm / pixelWidth / 10.0,
            bearingDeg = Math.toDegrees(atan2(centerX - GROUND_CAM_WIDTH / 2.0, C270_FOCAL_LENGTH_PIXELS)),
            screenX = centerX,
            screenY = centerY,
            contourArea = w * h,
        )
    }
}


/**
 * ============ BALL DETECTOR — tiny YOLO via TFLite, fully on-robot ============
 *
 * Neural net #2 (A of the staged plan). Attaches to the FRONT camera portal
 * alongside the HSV processors and runs a float-I/O TFLite detector at
 * NN_DETECT_HZ (default 3 — the hub CPU also runs the rest of vision, so
 * keep this LOW). Detections are published to [NeuralReferee], which the
 * BallLocator consults per the authority ladder (OFF / ADVISORY / REFEREE /
 * PRIMARY — see globals.kt).
 *
 * Train with tools/auto_label.py (+ tools/label_review.py to fix labels) and
 * tools/train_detector.py; copy ball_detector.tflite to the app's external
 * storage (the same place blackbox/ lives) over USB.
 */
class DetectorProcessor(context: Context) : VisionProcessor {

    private var interpreter: Interpreter? = null
    private var inputSize = 320
    private var inputBuf: ByteBuffer? = null
    private var outputShape: IntArray = IntArray(0)
    private var lastNanos = 0L
    private var paint: Paint? = null

    init {
        if (NN_DETECTOR_MODE != NnMode.OFF) {
            try {
                val f = File(context.getExternalFilesDir(null), NN_MODEL_FILE)
                if (!f.exists()) {
                    BlackBox.event("nn_status", "component" to "detector", "status" to "no model file")
                } else {
                    val channel = FileInputStream(f).channel
                    val model: MappedByteBuffer =
                        channel.map(FileChannel.MapMode.READ_ONLY, 0, f.length())
                    interpreter = Interpreter(model, Interpreter.Options().apply { numThreads = 2 })
                    val inShape = interpreter!!.getInputTensor(0).shape() // [1, S, S, 3]
                    inputSize = inShape[1]
                    inputBuf =
                        ByteBuffer.allocateDirect(inputSize * inputSize * 3 * 4)
                            .order(ByteOrder.nativeOrder())
                    outputShape = interpreter!!.getOutputTensor(0).shape()
                    BlackBox.event(
                        "nn_status", "component" to "detector",
                        "status" to "loaded", "inputSize" to inputSize,
                        "output" to outputShape.joinToString("x"),
                    )
                }
            } catch (e: Exception) {
                BlackBox.event(
                    "nn_status", "component" to "detector",
                    "status" to "error", "msg" to (e.message ?: "?"),
                )
            }
        }
    }

    val loaded: Boolean get() = interpreter != null

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {}

    override fun processFrame(frame: Mat?, captureTimeNanos: Long): Any? {
        val itp = interpreter ?: return null
        val f = frame ?: return null
        if (captureTimeNanos - lastNanos < (1e9 / NN_DETECT_HZ).toLong()) return null
        lastNanos = captureTimeNanos

        var resized: Mat? = null
        var rgb: Mat? = null
        return try {
            // 0. VisionProcessor frames are 4-channel RGBA (the SDK's own
            //    processors use COLOR_RGBA2GRAY/RGBA2RGB). Drop alpha FIRST —
            //    reading 3 bytes/pixel straight from an RGBA Mat yields
            //    misaligned garbage after the first pixel.
            val rgbMat = Mat()
            Imgproc.cvtColor(f, rgbMat, Imgproc.COLOR_RGBA2RGB)
            // 1. resize to the model's square input (aspect stretched back later)
            val resizedMat = Mat()
            Imgproc.resize(rgbMat, resizedMat, Size(inputSize.toDouble(), inputSize.toDouble()))
            resized = resizedMat
            rgb = rgbMat
            // 2. pixels -> float32 NHWC buffer in [0,1], RGB order like training
            val bytes = ByteArray(inputSize * inputSize * 3)
            resizedMat.get(0, 0, bytes)
            val buf = inputBuf!!
            buf.rewind()
            var i = 0
            while (i < bytes.size) {
                buf.putFloat((bytes[i].toInt() and 0xFF) / 255f)       // R
                buf.putFloat((bytes[i + 1].toInt() and 0xFF) / 255f)   // G
                buf.putFloat((bytes[i + 2].toInt() and 0xFF) / 255f)   // B
                i += 3
            }
            // 3. allocate output by the model's real shape and run
            val out = Array(outputShape[0]) {
                Array(outputShape[1]) { FloatArray(outputShape[2]) }
            }
            itp.run(buf, out)
            // 4. decode + NMS + publish (boxes scaled back to full-frame coords)
            NeuralReferee.publish(decode(out, f.cols().toDouble(), f.rows().toDouble()), captureTimeNanos)
            null
        } catch (_: Exception) {
            null // inference must never stall the vision pipeline
        } finally {
            // OpenCV Mats are native heap — always release or the 3 Hz
            // inference slowly leaks megabytes over a match.
            rgb?.release()
            resized?.release()
        }
    }

    /**
     * Decodes ultralytics-style TFLite output [1][4+nc][N] (cx, cy, w, h in
     * input pixels, then per-class scores) with greedy NMS. Boxes come back
     * in FULL-FRAME pixel coordinates.
     */
    private fun decode(out: Array<Array<FloatArray>>, frameW: Double, frameH: Double): List<NeuralReferee.Detection> {
        val channels = out[0].size
        val n = out[0][0].size
        val numClasses = channels - 4
        if (numClasses <= 0) return emptyList()

        val sx = frameW / inputSize
        val sy = frameH / inputSize
        val raw = ArrayList<DoubleArray>() // [cx, cy, w, h, score, cls]
        for (j in 0 until n) {
            var bestScore = 0.0
            var bestCls = -1
            for (c in 0 until numClasses) {
                val s = out[0][4 + c][j].toDouble()
                if (s > bestScore) {
                    bestScore = s; bestCls = c
                }
            }
            if (bestScore < NN_DETECT_CONFIDENCE) continue
            raw.add(doubleArrayOf(out[0][0][j] * sx, out[0][1][j] * sy,
                out[0][2][j] * sx, out[0][3][j] * sy, bestScore, bestCls.toDouble()))
        }
        raw.sortByDescending { it[4] }

        val kept = ArrayList<DoubleArray>()
        outer@ for (b in raw) {
            for (k in kept) {
                val x1 = maxOf(b[0] - b[2] / 2, k[0] - k[2] / 2)
                val y1 = maxOf(b[1] - b[3] / 2, k[1] - k[3] / 2)
                val x2 = minOf(b[0] + b[2] / 2, k[0] + k[2] / 2)
                val y2 = minOf(b[1] + b[3] / 2, k[1] + k[3] / 2)
                val inter = maxOf(0.0, x2 - x1) * maxOf(0.0, y2 - y1)
                if (inter / (b[2] * b[3] + k[2] * k[3] - inter) > NN_DETECT_IOU) continue@outer
            }
            kept.add(b)
        }

        return kept.take(12).map {
            NeuralReferee.Detection(
                labelId = it[5].toInt(),
                confidence = it[4],
                x = it[0] - it[2] / 2, y = it[1] - it[3] / 2,
                w = it[2], h = it[3],
            )
        }
    }

    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?,
    ) {
        val c = canvas ?: return
        val stroke = paint ?: Paint().apply {
            textSize = 24f
            isAntiAlias = true
            style = Paint.Style.STROKE
            strokeWidth = 3f
        }.also { paint = it }
        if (!NeuralReferee.fresh()) return
        val text = Paint().apply {
            textSize = 22f
            color = Color.rgb(0, 200, 255)
            style = Paint.Style.FILL
        }
        for (d in NeuralReferee.latest()) {
            stroke.color = Color.rgb(0, 200, 255)
            c.drawRect(
                (d.x * scaleBmpPxToCanvasPx).toFloat(),
                (d.y * scaleBmpPxToCanvasPx).toFloat(),
                ((d.x + d.w) * scaleBmpPxToCanvasPx).toFloat(),
                ((d.y + d.h) * scaleBmpPxToCanvasPx).toFloat(),
                stroke,
            )
            c.drawText(
                "NN %s %.0f%%".format(
                    org.firstinspires.ftc.teamcode.NN_LABELS.getOrElse(d.labelId) { "?" },
                    d.confidence * 100,
                ),
                (d.x * scaleBmpPxToCanvasPx).toFloat() + 4f,
                (d.y * scaleBmpPxToCanvasPx).toFloat() - 6f,
                text,
            )
        }
    }
}