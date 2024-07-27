package org.firstinspires.ftc.teamcode.stc.robot

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.lib.units.Pose
import org.firstinspires.ftc.teamcode.lib.units.cm
import org.firstinspires.ftc.teamcode.lib.units.deg
import org.firstinspires.ftc.teamcode.lib.units.inch
import org.firstinspires.ftc.teamcode.lib.units.rad
import org.firstinspires.ftc.teamcode.robot.ColorVisionProcessor
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.cos
import kotlin.math.sin

@Suppress("unused", "MemberVisibilityCanBePrivate")
class Camera(
    hardwareMap: HardwareMap,
    var telemetry: Telemetry? = null
) {
    enum class SIDE {
        BLUE,
        RED
    }

    val detectionProcessor = ColorVisionProcessor()
    val visionPortal: VisionPortal = VisionPortal.easyCreateWithDefaults(
        hardwareMap.get(WebcamName::class.java, "Webcam 1"),
        detectionProcessor
    )

    val detectionPosition: ColorVisionProcessor.DetectionPosition
        get() = detectionProcessor.analysis

    fun setColor(color: ColorVisionProcessor.DetectionColor) {
        detectionProcessor.setDetectionColor(color)
    }

    fun disableColorDetection() = visionPortal.setProcessorEnabled(detectionProcessor, false)

    fun displayDetection() {
        when (detectionPosition) {
            ColorVisionProcessor.DetectionPosition.LEFT -> telemetry?.addLine("LEFT CASE")
            ColorVisionProcessor.DetectionPosition.CENTER -> telemetry?.addLine("CENTER CASE")
            ColorVisionProcessor.DetectionPosition.RIGHT -> telemetry?.addLine("RIGHT CASE")
        }
    }

    fun stopStream() {
        visionPortal.stopStreaming()
    }

    fun resumeStream() {
        visionPortal.resumeStreaming()
    }
}