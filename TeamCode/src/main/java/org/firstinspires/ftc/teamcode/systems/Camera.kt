package org.firstinspires.ftc.teamcode.systems

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.lib.units.Distance2d
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.cm
import org.firstinspires.ftc.teamcode.lib.units.inch
import org.firstinspires.ftc.teamcode.lib.units.s
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.util.concurrent.TimeUnit
import kotlin.math.cos
import kotlin.math.sin

class Camera(
        hardwareMap: HardwareMap
) {
    val aprilTagProcessor = AprilTagProcessor.Builder()
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
            .setNumThreads(1)
            .build()

    private val visionPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .addProcessors(aprilTagProcessor)
            .build()

    private var init = true
    private var initTime = 0.s
    private var setExposure = false
    private var setGain = false

    private var expTime = 0.s

    fun runDetection(): Distance2d? {
        val detection = aprilTagProcessor.freshDetections?.firstOrNull { it.id in 1..6 } ?: return null

        val tagPose = getTagPose(detection.id)

        val angle = detection.ftcPose.yaw + tagPose.heading.toDouble()

        val yRobot = detection.ftcPose.y + 20.cm.inch

        val x = cos(angle) * yRobot - sin(angle) * detection.ftcPose.x
        val y = sin(angle) * yRobot + cos(angle) * detection.ftcPose.x

        return Distance2d(
                tagPose.position.x.inch - x.inch,
                tagPose.position.y.inch + y.inch
        )
    }

    fun getTagPose(tagId: Int): Pose2d {
        val aprilTagMetadata = AprilTagGameDatabase.getCenterStageTagLibrary().lookupTag(tagId)

        val orientation = aprilTagMetadata.fieldOrientation.toOrientation(AxesReference.EXTRINSIC, AxesOrder.YXZ, AngleUnit.RADIANS)

        return Pose2d(
                aprilTagMetadata.fieldPosition.data[0].toDouble(),
                aprilTagMetadata.fieldPosition.data[1].toDouble(),
                orientation.firstAngle.toDouble()
        )
    }

    fun init() {
        if (visionPortal.cameraState == VisionPortal.CameraState.STREAMING && init) {
            init = false
            initTime = Time.now()
            return
        }
        if (!setExposure && visionPortal.cameraState == VisionPortal.CameraState.STREAMING && Time.now() - initTime > 0.5.s) {
            val exposureControl = visionPortal.getCameraControl(ExposureControl::class.java)
            exposureControl.setMode(ExposureControl.Mode.Manual)
            exposureControl.setExposure(5, TimeUnit.MICROSECONDS)
            expTime = Time.now()

            setExposure = true
        }
        if (setExposure && !setGain && Time.now() - expTime > 0.5.s) {
            val gainControl = visionPortal.getCameraControl(GainControl::class.java)
            gainControl.setGain(100)

            setGain = true
        }
    }
}