package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.rotate

class Drive(hardwareMap: HardwareMap) {
    private val drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
    private val robotHeading
        get() = drive.pose.heading.log()
    private var speed = 1.0
    val sniperSpeed = 0.35
    var sniperMode = false
        set(value) {
            if (value == field)
                return
            speed = if (value) sniperSpeed else 1.0
            field = value
        }

    fun update() = drive.updatePoseEstimate()

    fun resetFieldCentric() {
        drive.pose = Pose2d(drive.pose.position, 0.0)
    }

    fun driveFieldCentric(x: Number, y: Number, heading: Number) = drive.setDrivePowers(PoseVelocity2d(Vector2d(x.toDouble(), y.toDouble()).rotate(-robotHeading - Math.toRadians(90.0)), heading.toDouble()).times(speed))

    private fun PoseVelocity2d.times(x: Double) : PoseVelocity2d {
        return PoseVelocity2d(this.linearVel.times(x), this.angVel * x)
    }
}