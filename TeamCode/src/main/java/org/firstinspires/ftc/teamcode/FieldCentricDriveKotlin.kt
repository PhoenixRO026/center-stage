package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@Disabled
@TeleOp
class FieldCentricDriveKotlin: LinearOpMode() {

    override fun runOpMode() {
        val drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))

        waitForStart()

        while (opModeIsActive()) {
            drive.updatePoseEstimate()

            val inputVector = Vector2d(gamepad1.left_stick_x.toDouble(), -gamepad1.left_stick_y.toDouble())
                    .rotate(-drive.pose.heading.log())

            val input = PoseVelocity2d (inputVector, gamepad1.right_stick_x.toDouble())

            drive.setDrivePowers(input)
        }
    }
}