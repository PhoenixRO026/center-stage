package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam

@Suppress("unused", "MemberVisibilityCanBePrivate")
class Camera(
    hardwareMap: HardwareMap,
    private val telemetry: Telemetry? = null
) {
    private val webcam: OpenCvWebcam
    private val pipeline = DetectionPipeline()

    val detectionPosition: DetectionPipeline.DetectionPosition
        get() = pipeline.analysis

    fun setColor(color: DetectionPipeline.DetectionColor) {
        pipeline.setDetectionColor(color)
    }

    fun displayDetection() {
        when (detectionPosition) {
            DetectionPipeline.DetectionPosition.LEFT -> telemetry?.addLine("LEFT CASE")
            DetectionPipeline.DetectionPosition.CENTER -> telemetry?.addLine("CENTER CASE")
            DetectionPipeline.DetectionPosition.RIGHT -> telemetry?.addLine("RIGHT CASE")
        }
    }

    fun stopStream() {
        webcam.stopStreaming()
    }

    fun openCamera() {
        webcam.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
            }

            override fun onError(errorCode: Int) {
                telemetry?.addLine("Failed to open camera")
                telemetry?.update()
            }
        })
    }

    init {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(
                WebcamName::class.java, "Webcam 1"
            ), cameraMonitorViewId
        )

        FtcDashboard.getInstance().startCameraStream(webcam, 30.0)

        webcam.setPipeline(pipeline)
    }
}