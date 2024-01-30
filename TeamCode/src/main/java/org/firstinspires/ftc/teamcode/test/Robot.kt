package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.lib.opmode.TimeListener
import org.firstinspires.ftc.teamcode.lib.opmode.Feature
import org.firstinspires.ftc.teamcode.lib.units.Pose
import org.firstinspires.ftc.teamcode.lib.units.cm
import org.firstinspires.ftc.teamcode.lib.units.deg
import org.firstinspires.ftc.teamcode.lib.units.rotate
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive

class Robot(hardwareMap: HardwareMap, time: TimeListener) : Feature, TimeListener by time {
    val drive = MecanumDrive(hardwareMap, Pose(0.cm, 0.cm, 0.deg).pose2d)

    fun fieldCentricDrive(gamepad: Gamepad) {
        val robotHeading = drive.imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)

        val strafe = Vector2d(
            -gamepad.left_stick_y.toDouble(),
            -gamepad.left_stick_x.toDouble()
        ).rotate(-robotHeading)

        val heading = -gamepad.right_stick_x.toDouble()

        drive.setDrivePowers(PoseVelocity2d(
            strafe,
            heading
        ))
    }
}