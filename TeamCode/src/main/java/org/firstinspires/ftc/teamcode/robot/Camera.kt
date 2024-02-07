package org.firstinspires.ftc.teamcode.robot

import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.VisionPortalImpl
import org.firstinspires.ftc.vision.VisionProcessor
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.openftc.easyopencv.OpenCvCamera

@Suppress("unused", "MemberVisibilityCanBePrivate")
class Camera(
    hardwareMap: HardwareMap,
    var telemetry: Telemetry? = null
) {
    val aprilTagProcessor: AprilTagProcessor = AprilTagProcessor.Builder()
        .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
        .build()
    //val aprilTagProcessor: AprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults()
    val detectionProcessor = ColorVisionProcessor()
    val visionPortal: VisionPortal = VisionPortal.easyCreateWithDefaults(
        hardwareMap.get(WebcamName::class.java, "Webcam 1"),
        detectionProcessor, detectionProcessor
    )

    val detectionPosition: ColorVisionProcessor.DetectionPosition
        get() = detectionProcessor.analysis

    fun setColor(color: ColorVisionProcessor.DetectionColor) {
        detectionProcessor.setDetectionColor(color)
    }

    fun disableColorDetection() = visionPortal.setProcessorEnabled(detectionProcessor, false)

    fun enableAprilTagDetection() = visionPortal.setProcessorEnabled(aprilTagProcessor, true)

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

    init {
        visionPortal.setProcessorEnabled(aprilTagProcessor, false)

        FtcDashboard.getInstance().startCameraStream((visionPortal as VisionPortalImplEx).getCamera(), 30.0)
    }
}