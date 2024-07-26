package org.firstinspires.ftc.teamcode.stc.teleop

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.stc.robot.Drive

@TeleOp
class DriveTele : LinearOpMode() {
    override fun runOpMode() {
        val drive = Drive(hardwareMap, Pose2d(0.0, 0.0, 0.0), Drive.Side.RED)

        waitForStart()

        while (opModeIsActive()){
            drive.speed = if (gamepad1.left_trigger >= 0.2) 0.5 else 1.0

            drive.drive(
                    -gamepad1.left_stick_y.toDouble(),
                    gamepad1.left_stick_x.toDouble(),
                    gamepad1.right_stick_x.toDouble()
            )
        }
    }

}