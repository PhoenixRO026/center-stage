package org.firstinspires.ftc.teamcode.stc.robot

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.lib.units.rotate
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive

class Drive (hardwareMap: HardwareMap, startPose: Pose2d, side: Side){
    private val mechanumDrive = MecanumDrive(hardwareMap, startPose)

    private val offSet = when(side){
        Side.BLUE -> Math.toRadians(-90.0)
        Side.RED -> Math.toRadians(90.0)
        Side.NEUTRAl -> 0.0
    }

    var speed = 1.0

    fun drive(forward : Double, strafe : Double, rotate : Double) {
        mechanumDrive.updatePoseEstimate()

        val driveVector = Vector2d(forward, strafe)
        val rotatedVector = driveVector.rotate(offSet - mechanumDrive.pose.heading.toDouble())

        val drivePower = PoseVelocity2d(rotatedVector * speed, rotate * speed)

        mechanumDrive.setDrivePowers(drivePower)
    }

    enum class Side {
        BLUE,
        RED,
        NEUTRAl
    }
}