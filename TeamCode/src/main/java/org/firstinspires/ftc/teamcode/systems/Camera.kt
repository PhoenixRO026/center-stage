package org.firstinspires.ftc.teamcode.systems

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.phoenix.phoenixlib.units.Distance2d
import com.phoenix.phoenixlib.units.SleepAction
import com.phoenix.phoenixlib.units.cm
import com.phoenix.phoenixlib.units.inch
import com.phoenix.phoenixlib.units.s
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.robotcore.external.matrices.VectorF
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion
import org.firstinspires.ftc.teamcode.lib.vision.ColorVisionProcessor
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.util.concurrent.TimeUnit
import kotlin.math.cos
import kotlin.math.sin

class Camera(
    hardwareMap: HardwareMap,
    var telemetry: Telemetry? = null
) {
    @Config
    data object CameraConfig {
        @JvmField var cameraOffset = 13.0
        @JvmField var cameraExposure: Long = 1
        @JvmField var cameraGain = 230
    }

    private val aprilTagLibrary = AprilTagLibrary.Builder()
        .addTag(
            1, "BlueAllianceLeft",
            2.0, VectorF(83.86f, 41.41f, 4f), DistanceUnit.INCH,
            Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0)
        )
        .addTag(
            2, "BlueAllianceCenter",
            2.0, VectorF(83.86f, 35.41f, 4f), DistanceUnit.INCH,
            Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0)
        )
        .addTag(
            3, "BlueAllianceRight",
            2.0, VectorF(83.86f, 29.41f, 4f), DistanceUnit.INCH,
            Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0)
        )
        .addTag(
            4, "RedAllianceLeft",
            2.0, VectorF(83.86f, -29.41f, 4f), DistanceUnit.INCH,
            Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0)
        )
        .addTag(
            5, "RedAllianceCenter",
            2.0, VectorF(83.86f, -35.41f, 4f), DistanceUnit.INCH,
            Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0)
        )
        .addTag(
            6, "RedAllianceRight",
            2.0, VectorF(83.86f, -41.41f, 4f), DistanceUnit.INCH,
            Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0)
        )
        .addTag(
            7, "RedAudienceWallLarge",
            5.0, VectorF(-70.25f, -40.625f, 5.5f), DistanceUnit.INCH,
            Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0)
        )
        .addTag(
            8, "RedAudienceWallSmall",
            2.0, VectorF(-70.25f, -35.125f, 4f), DistanceUnit.INCH,
            Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0)
        )
        .addTag(
            9, "BlueAudienceWallSmall",
            2.0, VectorF(-70.25f, 35.125f, 4f), DistanceUnit.INCH,
            Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0)
        )
        .addTag(
            10, "BlueAudienceWallLarge",
            5.0, VectorF(-70.25f, 40.625f, 5.5f), DistanceUnit.INCH,
            Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0)
        )
        .addTag(
            11, "SharedBackdropLeft",
            2.0, VectorF(83.86f, 6.0f, 4f), DistanceUnit.INCH,
            Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0)
        )
        .addTag(
            12, "SharedBackdropCenter",
            2.0, VectorF(83.86f, 0.0f, 4f), DistanceUnit.INCH,
            Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0)
        )
        .addTag(
            13, "SharedBackdropRight",
            2.0, VectorF(83.86f, -6.0f, 4f), DistanceUnit.INCH,
            Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0)
        )
        .build()

    private val aprilTagProcessor = AprilTagProcessor.Builder()
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
            .setNumThreads(1)
            .setTagLibrary(aprilTagLibrary)
            .build()

    private val detectionProcessor = ColorVisionProcessor()

    private val visionPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .addProcessors(aprilTagProcessor, detectionProcessor)
            //.enableLiveView(false)
            .build()

    private lateinit var exposureControl: ExposureControl

    private lateinit var gainControl: GainControl

    val exposureAction = SequentialAction(
        Action { visionPortal.cameraState != VisionPortal.CameraState.STREAMING },
        SleepAction(0.5.s),
        InstantAction {
            exposureControl = visionPortal.getCameraControl(ExposureControl::class.java)
            gainControl = visionPortal.getCameraControl(GainControl::class.java)
            exposureControl.setMode(ExposureControl.Mode.Manual)
        },
        SleepAction(0.5.s),
        InstantAction {
            exposureControl.setExposure(CameraConfig.cameraExposure, TimeUnit.MILLISECONDS)
        },
        SleepAction(0.5.s),
        InstantAction {
            gainControl.setGain(CameraConfig.cameraGain)
        }
    )

    val detectionPosition: ColorVisionProcessor.DetectionPosition
        get() = detectionProcessor.analysis

    fun setColor(color: ColorVisionProcessor.DetectionColor) {
        detectionProcessor.setDetectionColor(color)
    }

    fun disableColorDetection() = visionPortal.setProcessorEnabled(detectionProcessor, false)

    fun enableAprilTagDetection() = visionPortal.setProcessorEnabled(aprilTagProcessor, true)

    fun disableAprilTagDetection() = visionPortal.setProcessorEnabled(aprilTagProcessor, false)

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

    private var initNotDone = true
    private var lowerExposure = false

    fun lowerExposure() {
        lowerExposure = true
    }

    fun runDetection(): Distance2d? {
        val detection = aprilTagProcessor.freshDetections?.firstOrNull { it.id in 1..6 || it.id in 11..13 } ?: return null

        val tagPose = getTagPose(detection.id)

        val angle = detection.ftcPose.yaw + tagPose.heading.toDouble()

        val yRobot = detection.ftcPose.y + CameraConfig.cameraOffset.cm.inch

        val x = cos(angle) * yRobot - sin(angle) * detection.ftcPose.x
        val y = sin(angle) * yRobot + cos(angle) * detection.ftcPose.x

        return Distance2d(
                tagPose.position.x.inch - x.inch,
                tagPose.position.y.inch + y.inch
        )
    }

    fun getTagPose(tagId: Int): Pose2d {
        val aprilTagMetadata = aprilTagLibrary.lookupTag(tagId)

        val orientation = aprilTagMetadata.fieldOrientation.toOrientation(AxesReference.EXTRINSIC, AxesOrder.YXZ, AngleUnit.RADIANS)

        return Pose2d(
                aprilTagMetadata.fieldPosition.data[0].toDouble(),
                aprilTagMetadata.fieldPosition.data[1].toDouble(),
                orientation.firstAngle.toDouble()
        )
    }

    init {
        disableAprilTagDetection()
        //disableColorDetection()
    }

    fun update() {
        if (lowerExposure) {
            if (initNotDone) {
                initNotDone = exposureAction.run(TelemetryPacket())
            }
        }
    }
}