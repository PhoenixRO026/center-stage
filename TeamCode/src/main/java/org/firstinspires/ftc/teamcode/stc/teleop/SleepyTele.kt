package org.firstinspires.ftc.teamcode.stc.teleop

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.stc.robot.Arm
import org.firstinspires.ftc.teamcode.stc.robot.Claw
import org.firstinspires.ftc.teamcode.stc.robot.Drive
import org.firstinspires.ftc.teamcode.stc.robot.Intake
import org.firstinspires.ftc.teamcode.stc.robot.Lift

@TeleOp
class SleepyTele: LinearOpMode() {
    override fun runOpMode() {
        val lift = Lift(hardwareMap)
        val claw = Claw(hardwareMap)
        val arm = Arm(hardwareMap)
        val intake = Intake(hardwareMap)
        val drive = Drive(hardwareMap, Pose2d(0.0, 0.0, 0.0), Drive.Side.NEUTRAL)
        val plane = Plane(hardwareMap)

        claw.tilt = Claw.intakeTilt
        arm.pos = Arm.intakePos

        waitForStart()

        while (opModeIsActive()) {
            if (gamepad2.left_bumper) {
                claw.tilt = Claw.intakeTilt
                arm.pos = Arm.intakePos
            } else if (gamepad2.right_bumper) {
                claw.tilt = Claw.scoreTilt
                arm.pos = Arm.scorePos
            }

            claw.leftFinger = gamepad2.right_trigger.toDouble()
            claw.rightFinger = gamepad2.left_trigger.toDouble()

            intake.power = -gamepad2.left_stick_y.toDouble()

            lift.power = -gamepad2.right_stick_y.toDouble()

            if (gamepad2.back) {
                lift.hang()
            } else if (gamepad2.start) {
                lift.unhang()
            }

            drive.speed = if (gamepad1.left_trigger >= 0.2) 0.5 else 1.0

            if (gamepad1.y) drive.resetHeading()

            drive.drive(
                -gamepad1.left_stick_y.toDouble(),
                gamepad1.left_stick_x.toDouble(),
                gamepad1.right_stick_x.toDouble()
            )

            if (gamepad2.b) plane.launch()

            plane.update()

            telemetry.addData("heading", Math.toDegrees(drive.mechanumDrive.pose.heading.toDouble()))
            telemetry.update()
        }
    }
}