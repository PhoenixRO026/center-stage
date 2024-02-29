package org.firstinspires.ftc.teamcode.lib.systems

import com.acmerobotics.dashboard.FtcDashboard
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

    val aprilTagProcessor: AprilTagProcessor = AprilTagProcessor.Builder()
        .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
        .setNumThreads(1)
        .build()
    //val aprilTagProcessor: AprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults()
    val detectionProcessor = ColorVisionProcessor()
    val visionPortal: VisionPortal = VisionPortal.easyCreateWithDefaults(
        hardwareMap.get(WebcamName::class.java, "Webcam 1"),
        detectionProcessor, aprilTagProcessor
    )
    val aprilTag1Pose: Pose
    val aprilTag2Pose: Pose
    val aprilTag3Pose: Pose
    val aprilTag5Pose: Pose
    val aprilTag4Pose: Pose
    val aprilTag6Pose: Pose

    var robotPose = Pose(0.cm, 0.cm, 0.deg)
        private set

    fun findTag5() = findTag(SIDE.RED)

    fun findTag2() = findTag(SIDE.BLUE)

    private fun findTag(side: SIDE): Boolean {
        val redTag = aprilTagProcessor.freshDetections?.firstOrNull {
            when(side) {
                SIDE.BLUE -> it.id == 1 || it.id == 2 || it.id == 3
                SIDE.RED -> it.id == 5 || it.id == 4 || it.id == 6
            }
        } ?: return false
        val redPose = redTag.ftcPose
        val fieldPose = when(redTag.id) {
            1 -> aprilTag1Pose
            2 -> aprilTag2Pose
            3 -> aprilTag3Pose
            5 -> aprilTag5Pose
            4 -> aprilTag4Pose
            6 -> aprilTag6Pose
            else -> aprilTag5Pose
        }

        val angle = redPose.yaw + aprilTag5Pose.heading.rad

        val yRobot = redPose.y + 20.cm.inch

        val x = cos(angle) * yRobot - sin(angle) * redPose.x
        val y = sin(angle) * yRobot + cos(angle) * redPose.x

        robotPose = Pose(
            fieldPose.position.x - x.inch,
            fieldPose.position.y + y.inch,
            fieldPose.heading - redPose.yaw.rad - 180.deg
        )

        return true
    }

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

    init {
        visionPortal.setProcessorEnabled(aprilTagProcessor, false)

        //FtcDashboard.getInstance().startCameraStream(visionPortal, 30.0)

        val aprilTag1: AprilTagMetadata = AprilTagGameDatabase.getCenterStageTagLibrary().lookupTag(1)

        val orientation1 = aprilTag1.fieldOrientation.toOrientation(AxesReference.EXTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES)

        aprilTag1Pose = Pose(
            aprilTag1.fieldPosition.data[0].inch,
            aprilTag1.fieldPosition.data[1].inch,
            orientation1.firstAngle.deg
        )

        val aprilTag2: AprilTagMetadata = AprilTagGameDatabase.getCenterStageTagLibrary().lookupTag(2)

        val orientation2 = aprilTag2.fieldOrientation.toOrientation(AxesReference.EXTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES)

        aprilTag2Pose = Pose(
            aprilTag2.fieldPosition.data[0].inch,
            aprilTag2.fieldPosition.data[1].inch,
            orientation2.firstAngle.deg
        )

        val aprilTag3: AprilTagMetadata = AprilTagGameDatabase.getCenterStageTagLibrary().lookupTag(3)

        val orientation3 = aprilTag3.fieldOrientation.toOrientation(AxesReference.EXTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES)

        aprilTag3Pose = Pose(
            aprilTag3.fieldPosition.data[0].inch,
            aprilTag3.fieldPosition.data[1].inch,
            orientation3.firstAngle.deg
        )

        val aprilTag4: AprilTagMetadata = AprilTagGameDatabase.getCenterStageTagLibrary().lookupTag(4)

        val orientation4 = aprilTag4.fieldOrientation.toOrientation(AxesReference.EXTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES)

        aprilTag4Pose = Pose(
            aprilTag4.fieldPosition.data[0].inch,
            aprilTag4.fieldPosition.data[1].inch,
            orientation4.firstAngle.deg
        )

        val aprilTag5: AprilTagMetadata = AprilTagGameDatabase.getCenterStageTagLibrary().lookupTag(5)

        val orientation5 = aprilTag5.fieldOrientation.toOrientation(AxesReference.EXTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES)

        aprilTag5Pose = Pose(
            aprilTag5.fieldPosition.data[0].inch,
            aprilTag5.fieldPosition.data[1].inch,
            orientation5.firstAngle.deg
        )

        val aprilTag6: AprilTagMetadata = AprilTagGameDatabase.getCenterStageTagLibrary().lookupTag(6)

        val orientation6 = aprilTag6.fieldOrientation.toOrientation(AxesReference.EXTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES)

        aprilTag6Pose = Pose(
            aprilTag6.fieldPosition.data[0].inch,
            aprilTag6.fieldPosition.data[1].inch,
            orientation6.firstAngle.deg
        )
    }
}