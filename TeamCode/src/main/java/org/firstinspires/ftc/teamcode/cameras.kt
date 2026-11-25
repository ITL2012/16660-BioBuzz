package org.firstinspires.ftc.teamcode

import android.util.Size
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor
import org.firstinspires.ftc.vision.opencv.ColorRange
import org.opencv.core.*
import java.util.concurrent.TimeUnit

/**
 * VisionSystem handles AprilTag detection for Hives and Color Blob detection for Pollen/Nectar/Flowers.
 * It manages two camera portals and provides localized detection data with visual overlays.
 */
class VisionSystem(hardwareMap: HardwareMap) {

    // 1. Hardware Initialization
    val groundCam = hardwareMap.get(WebcamName::class.java, GROUND_CAM_NAME)
    val highCam = hardwareMap.get(WebcamName::class.java, HIGH_CAM_NAME)

    // 2. Processor Setup
    val aprilTagProcessor = AprilTagProcessor.Builder()
        .setDrawAxes(true)
        .setDrawCubeProjection(true)
        .setDrawTagOutline(true)
        .build()

    val pollenLocator = createLocator(POLLEN_HSV_RANGE)
    val nectarBlueLocator = createLocator(NECTAR_BLUE_HSV_RANGE)
    val nectarRedLocator = createLocator(NECTAR_RED_HSV_RANGE)
    val nectarRedWrapLocator = createLocator(NECTAR_RED_WRAP_HSV_RANGE)
    val flowerLocator = createLocator(FLOWER_GREEN_HSV_RANGE)

    /**
     * Robot detection: opposing robots are large, mostly-dark shapes.
     * We look for dark blobs (black plastic + shadows) — bigger than any game piece.
     * Filtered by contour area at read-time so small noise doesn't read as a robot.
     */
    val robotDetector = ColorBlobLocatorProcessor.Builder()
        .setTargetColorRange(ROBOT_DARK_HSV_RANGE)
        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
        .setDrawContours(DEBUG_MODE)
        .build()

    private fun createLocator(range: ColorRange): ColorBlobLocatorProcessor {
        return ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(range)
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
            .setDrawContours(DEBUG_MODE)
            .build()
    }

    // Custom Overlay and Calibration Processors (Moved to debug.kt)
    val highMetadataOverlay = MetadataOverlay()
    val groundMetadataOverlay = MetadataOverlay()
    val calibrationProcessor = CalibrationProcessor()

    // 3. Vision Portal Setup
    private val portalViewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL)

    val highPortal = VisionPortal.Builder()
        .setCamera(highCam)
        .setCameraResolution(Size(CAMERA_WIDTH_PIXELS, CAMERA_HEIGHT_PIXELS))
        .setLiveViewContainerId(portalViewIds[0])
        .addProcessors(aprilTagProcessor, highMetadataOverlay)
        .build()

    val groundPortal = VisionPortal.Builder()
        .setCamera(groundCam)
        .setCameraResolution(Size(GROUND_CAM_WIDTH, GROUND_CAM_HEIGHT))
        .setLiveViewContainerId(portalViewIds[1])
        .addProcessors(pollenLocator, nectarBlueLocator, nectarRedLocator, nectarRedWrapLocator, flowerLocator, groundMetadataOverlay, calibrationProcessor)
        .build()

    init {
        // Disable debug processors if not in debug mode
        if (!DEBUG_MODE) {
            highPortal.setProcessorEnabled(highMetadataOverlay, false)
            groundPortal.setProcessorEnabled(groundMetadataOverlay, false)
            groundPortal.setProcessorEnabled(calibrationProcessor, false)
        } else {
            // Even in debug mode, keep calibration off by default
            groundPortal.setProcessorEnabled(calibrationProcessor, false)
        }
    }

    /**
     * Data class for standardized detection results.
     */
    data class Detection(
        val label: String,
        val distanceMm: Double,
        val angleDegrees: Double,
        val xMm: Double = 0.0,
        val yMm: Double = 0.0,
        val zMm: Double = 0.0,
        val metadata: String = "",
        val screenX: Float = 0f,
        val screenY: Float = 0f
    )

    /**
     * Returns all detections from both portals and updates overlays.
     */
    fun getAllDetections(): List<Detection> {
        val highDetections = mutableListOf<Detection>()
        val groundDetections = mutableListOf<Detection>()

        // 1. Process AprilTags (High Portal)
        // SDK 12.0: official BioBuzz hives are 4-tag CLUSTERS whose origin is the
        // Cell-opening center — the cluster pose IS the aim point. The stock
        // BioBuzz library contains NO single tags, so clusters are the normal
        // path; the single-tag branch remains for custom/single-tag libraries.
        val tags = aprilTagProcessor.detections
        for (tag in tags) {
            when (tag) {
                is AprilTagClusterDetection -> {
                    // Only OUR alliance's hives count (SELECTED_TEAM, globals.kt).
                    if (tag.metadata?.shortName !in teamClusterNames(SELECTED_TEAM)) continue
                    val ftc = tag.ftcPose ?: continue
                    highDetections.add(
                        Detection(
                            label = "Hive Cluster ${tag.metadata.shortName}",
                            distanceMm = ftc.range * 25.4,
                            // SDK ftc.bearing is + = LEFT; normalize to + = right
                            // to match the blob detections' angle convention.
                            angleDegrees = -ftc.bearing,
                            xMm = ftc.x * 25.4,
                            yMm = ftc.y * 25.4,
                            zMm = ftc.z * 25.4,
                            metadata = "${tag.percentClusterFound}% tags found",
                        )
                    )
                }
                is AprilTagSingleDetection -> {
                    if (tag.metadata == null) continue
                    // Only OUR alliance's hives count (SELECTED_TEAM, globals.kt).
                    if (tag.id !in teamTagIds(SELECTED_TEAM)) continue
                    // Uncalibrated cameras can detect a tag without a pose.
                    val ftc = tag.ftcPose ?: continue
                    highDetections.add(
                        Detection(
                            label = "Hive Tag ${tag.id}",
                            distanceMm = ftc.range * 25.4,
                            // SDK ftc.bearing is + = LEFT; normalize to + = right
                            // to match the blob detections' angle convention.
                            angleDegrees = -ftc.bearing,
                            xMm = ftc.x * 25.4,
                            yMm = ftc.y * 25.4,
                            zMm = ftc.z * 25.4,
                            metadata = "ID ${tag.id}",
                            screenX = tag.center.x.toFloat(),
                            screenY = tag.center.y.toFloat()
                        )
                    )
                }
            }
        }

        // 2. Process Blobs (Ground Portal)
        addBlobDetections(groundDetections, pollenLocator.blobs, "Pollen", POLLEN_PHYSICAL_WIDTH_MM)
        addBlobDetections(groundDetections, nectarBlueLocator.blobs, "Nectar Blue", NECTAR_PHYSICAL_WIDTH_MM)
        addBlobDetections(groundDetections, nectarRedLocator.blobs, "Nectar Red", NECTAR_PHYSICAL_WIDTH_MM)
        addBlobDetections(groundDetections, nectarRedWrapLocator.blobs, "Nectar Red", NECTAR_PHYSICAL_WIDTH_MM)
        addBlobDetections(groundDetections, flowerLocator.blobs, "Flower", FLOWER_PHYSICAL_WIDTH_MM)

        // 3. Process Robots (Ground Portal)
        // Only accept blobs big enough to plausibly be a robot — filtered by contour area.
        val robotBlobs = robotDetector.blobs.toMutableList()
        ColorBlobLocatorProcessor.Util.filterByCriteria(
            ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
            MIN_ROBOT_CONTOUR_AREA, Double.MAX_VALUE, robotBlobs
        )
        addBlobDetections(groundDetections, robotBlobs, "Robot", ROBOT_PHYSICAL_WIDTH_MM)

        if (DEBUG_MODE) {
            highMetadataOverlay.updateDetections(highDetections)
            groundMetadataOverlay.updateDetections(groundDetections)
        }

        return highDetections + groundDetections
    }

    private fun addBlobDetections(
        list: MutableList<Detection>,
        blobs: List<ColorBlobLocatorProcessor.Blob>,
        label: String,
        physicalWidthMm: Double
    ) {
        for (blob in blobs) {
            val box = blob.boxFit
            val pixelWidth = Math.max(box.size.width, box.size.height)
            
            val distanceMm = (C270_FOCAL_LENGTH_PIXELS * physicalWidthMm) / pixelWidth
            val centerX = box.center.x
            val relativeX = centerX - (GROUND_CAM_WIDTH / 2.0)
            val angleDegrees = Math.toDegrees(Math.atan2(relativeX, C270_FOCAL_LENGTH_PIXELS))

            list.add(
                Detection(
                    label = label,
                    distanceMm = distanceMm,
                    angleDegrees = angleDegrees,
                    metadata = "${pixelWidth.toInt()}px",
                    screenX = box.center.x.toFloat(),
                    screenY = box.center.y.toFloat()
                )
            )
        }
    }

    fun close() {
        highPortal.close()
        groundPortal.close()
    }

}