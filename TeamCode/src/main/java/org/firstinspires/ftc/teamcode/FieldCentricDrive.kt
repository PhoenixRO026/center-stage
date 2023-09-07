package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class FieldCentricDrive: LinearOpMode() {

    private lateinit var drive: MecanumDrive
    override fun runOpMode() {
        drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))

        waitForStart()

        while (opModeIsActive()) {
            drive.updatePoseEstimate()

            val input = PoseVelocity2d (
                    linearVel = Vector2d (
                            x = gamepad1.left_stick_x.toDouble(),
                            y = -gamepad1.left_stick_y.toDouble()
                        ).rotate(drive.pose.heading.log()),
                    angVel = gamepad1.right_stick_x.toDouble(),
            )

            drive.setDrivePowers(input)
        }
    }
}