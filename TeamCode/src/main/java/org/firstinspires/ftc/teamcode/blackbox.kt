package org.firstinspires.ftc.teamcode

/**
 * ============ BLACK BOX — passive full-match recorder + frame capture ============
 *
 * One merged home for the recorder pair (was the blackbox/ package):
 *  * [BlackBox]           — JSON-lines state log on the Robot Controller
 *  * [FrameGrabProcessor] — subsampled JPEG frames from both cameras
 *
 * Legality posture: 100% passive, 100% onboard, no new hardware — see
 * docs/LEGALITY-REVIEW.md.
 */

import android.content.Context
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.json.JSONObject
import org.opencv.core.Mat
import org.opencv.imgcodecs.Imgcodecs
import org.opencv.imgproc.Imgproc
import java.io.BufferedWriter
import java.io.File
import java.io.FileWriter


/**
 * ============ BLACK BOX — passive full-match data recorder ============
 *
 * Writes one JSON-lines file (state.jsonl) plus JPEG frame folders per OpMode
 * run to the ROBOT CONTROLLER's own external storage. One line = one record:
 * pose, vision sightings, world-model tracks, planner decisions, flywheel
 * RPMs, motor currents, battery voltage, gamepad inputs, events. The
 * FrameGrabProcessor adds timestamped JPEG frames from BOTH robot cameras.
 *
 * LEGALITY POSTURE (see docs/LEGALITY-REVIEW.md):
 *  * 100% PASSIVE — reads state, never influences control.
 *  * 100% ONBOARD — writes only to the Robot Controller's storage; no
 *    streaming, no radio, no internet, nothing leaves the robot mid-match.
 *  * No new hardware — uses the two cameras that are already attached to the
 *    Robot Controller for the vision stack.
 * After the match, copy `blackbox/` off the phone over USB and run
 * tools/blackbox_builder.py on a computer.
 *
 * Files land in: /sdcard/Android/data/<rc-app>/files/blackbox/<label>_<ts>/
 */
object BlackBox {

    /** True while a recording session is active. Cheap to poll from anywhere. */
    @Volatile
    var active = false
        private set

    private var writer: BufferedWriter? = null
    private var sessionDir: File? = null
    private val frameDirs = HashMap<String, File>()
    private var linesSinceFlush = 0
    private var droppedLines = 0L

    /** Approximate bytes written by frame JPEGs this session (budget guard). */
    @Volatile
    var frameBytesSaved = 0L
        private set

    /**
     * Starts a new session. Call once at the top of runOpMode().
     * [label] names the folder, e.g. "teleop" / "auto" / "selfplay".
     * Returns false (and does nothing) when BLACKBOX_ENABLED is false or
     * storage is unavailable.
     */
    @Synchronized
    fun start(context: Context, label: String): Boolean {
        if (!BLACKBOX_ENABLED) return false
        stop()
        return try {
            val root = File(context.getExternalFilesDir(null) ?: context.filesDir, "blackbox")
            root.mkdirs()
            val dir = File(root, "${label}_${System.currentTimeMillis()}")
            dir.mkdirs()
            sessionDir = dir
            writer = BufferedWriter(FileWriter(File(dir, "state.jsonl")))
            active = true
            frameBytesSaved = 0
            event("session_start", "label" to label, "version" to 1)
            true
        } catch (_: Exception) {
            active = false
            false
        }
    }

    /**
     * Appends one state line. Values must be JSON-friendly primitives or
     * nested Maps/Lists. Never throws — logging must be able to fail silently
     * rather than hurt the control loop. Calls are serialized so the OpMode
     * loop and vision threads can both log safely.
     */
    @Synchronized
    fun log(vararg fields: Pair<String, Any?>) {
        val w = writer ?: return
        try {
            val o = JSONObject()
            o.put("t", System.currentTimeMillis())
            o.put("tn", System.nanoTime())
            for ((k, v) in fields) o.put(k, v)
            w.write(o.toString())
            w.write("\n")
            if (++linesSinceFlush >= 20) {
                w.flush()
                linesSinceFlush = 0
            }
        } catch (_: Exception) {
            droppedLines++
        }
    }

    /** A state line tagged with an event type — phase changes, shots, jams... */
    @Synchronized
    fun event(type: String, vararg fields: Pair<String, Any?>) {
        log("event" to type, *fields)
        try {
            writer?.flush()
        } catch (_: Exception) {
        }
    }

    /** Session folder for one camera's frames, or null when inactive. */
    @Synchronized
    fun framesDirFor(camera: String): File? {
        if (!active) return null
        val dir = sessionDir ?: return null
        return frameDirs.getOrPut(camera) {
            File(File(dir, "frames"), camera).apply { mkdirs() }
        }
    }

    /** False once the frame budget is exhausted (state lines keep logging). */
    fun frameBudgetAvailable(): Boolean =
        active && frameBytesSaved < BLACKBOX_MAX_SESSION_MB * 1024.0 * 1024.0

    /** Called by FrameGrabProcessor after a successful JPEG write. */
    @Synchronized
    fun noteFrame(camera: String, file: String, captureNanos: Long, bytes: Int) {
        frameBytesSaved += bytes
        log("frame" to camera, "file" to file, "captureNanos" to captureNanos)
    }

    /** Ends the session and flushes everything to disk. Call in finally {}. */
    @Synchronized
    fun stop() {
        if (!active && writer == null) {
            sessionDir = null
            frameDirs.clear()
            return
        }
        try {
            writer?.flush()
            writer?.close()
        } catch (_: Exception) {
        }
        if (droppedLines > 0) {
            // best effort: record the counter loss in the folder itself
            try {
                sessionDir?.let { File(it, "DROPPED_LINES.txt").writeText("$droppedLines\n") }
            } catch (_: Exception) {
            }
        }
        writer = null
        sessionDir = null
        frameDirs.clear()
        active = false
        droppedLines = 0
    }
}


/**
 * Attaches to a VisionPortal alongside the normal vision processors and saves
 * subsampled JPEG frames from the raw camera stream to the Black Box session.
 * The frames are the EXACT images the vision stack saw — timestamps
 * (captureNanos) match the state.jsonl lines, so a computer can replay what
 * the robot "was thinking" frame by frame.
 *
 * Cost control: BLACKBOX_FRAME_CAPTURE_FPS per camera (default 5) and the session
 * MB budget in globals.kt. A bounded worker per camera owns JPEG encoding and
 * disk I/O; if it falls behind, capture frames are dropped instead of blocking
 * the vision thread.
 */
class FrameGrabProcessor(private val camera: String) : VisionProcessor, AutoCloseable {

    @Volatile
    var enabled = true

    private var lastSaveNanos = 0L
    private var frameCount = 0
    private val queueLock = Object()
    private var pending: Capture? = null
    private var recycledFrame: Mat? = null
    private var worker: Thread? = null

    @Volatile
    private var closed = false

    private data class Capture(
        val rgba: Mat,
        val dir: File,
        val name: String,
        val captureNanos: Long,
    )

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
        if (worker == null) {
            worker = Thread({ writeLoop() }, "BlackBox-$camera-JPEG").apply {
                isDaemon = true
                start()
            }
        }
    }

    override fun processFrame(frame: Mat?, captureTimeNanos: Long): Any? {
        val f = frame ?: return null
        if (closed || !enabled || !BlackBox.active) return null
        val intervalNanos = (1e9 / BLACKBOX_FRAME_CAPTURE_FPS).toLong()
        if (captureTimeNanos - lastSaveNanos < intervalNanos) return null
        synchronized(queueLock) {
            if (pending != null) return null
        }
        val dir = BlackBox.framesDirFor(camera) ?: return null
        if (!BlackBox.frameBudgetAvailable()) return null

        synchronized(queueLock) {
            if (closed || pending != null) return null
            val copy = recycledFrame?.also { recycledFrame = null } ?: Mat()
            try {
                f.copyTo(copy)
            } catch (_: Exception) {
                copy.release()
                return null
            }
            pending = Capture(copy, dir, "frame_%06d.jpg".format(++frameCount), captureTimeNanos)
            lastSaveNanos = captureTimeNanos
            queueLock.notifyAll()
        }
        return null
    }

    private fun writeLoop() {
        while (true) {
            val capture = synchronized(queueLock) {
                while (pending == null && !closed) queueLock.wait()
                if (closed) return@synchronized null
                pending.also { pending = null }
            } ?: break

            var bgr: Mat? = null
            try {
                val bgrMat = Mat()
                bgr = bgrMat
                Imgproc.cvtColor(capture.rgba, bgrMat, Imgproc.COLOR_RGBA2BGR)
                val out = File(capture.dir, capture.name)
                if (Imgcodecs.imwrite(out.absolutePath, bgrMat)) {
                    BlackBox.noteFrame(camera, capture.name, capture.captureNanos, out.length().toInt())
                }
            } catch (_: Exception) {
            } finally {
                bgr?.release()
                recycle(capture.rgba)
            }
        }
    }

    private fun recycle(frame: Mat) {
        synchronized(queueLock) {
            if (!closed && recycledFrame == null) {
                recycledFrame = frame
                return
            }
        }
        frame.release()
    }

    override fun close() {
        val thread = synchronized(queueLock) {
            if (closed) return@synchronized null
            closed = true
            pending?.rgba?.release()
            pending = null
            recycledFrame?.release()
            recycledFrame = null
            queueLock.notifyAll()
            worker
        }
        thread?.join(2000)
        worker = null
    }

    override fun onDrawFrame(
        canvas: android.graphics.Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?,
    ) {
    }
}
/**
 * SUPERHUMAN-ONLY: dumps the core [WorldModel]'s snapshot to the Black Box
 * (~10 Hz). The core class stays Black-Box-free (identical to master); the
 * branch adds the logging as an extension so every call site is unchanged.
 */
fun WorldModel.logToBlackBox() {
    BlackBox.log(*(listOf("record" to "wm") + logPayload().map { (k, v) -> k to v }).toTypedArray())
}
