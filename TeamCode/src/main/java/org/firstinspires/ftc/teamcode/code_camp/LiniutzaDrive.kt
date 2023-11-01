package org.firstinspires.ftc.teamcode.code_camp

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.ButtonReader
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.code_camp.robot.Robot

@TeleOp
class LiniutzaDrive : LinearOpMode(){

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val liftLimitButton = ButtonReader { gamepad1.x || gamepad2.x }
        val resetHeadingButton = ButtonReader { gamepad1.b }
        val robot = Robot(hardwareMap, telemetry)

        waitForStart()

        while (opModeIsActive()) {
            robot.update()

            liftLimitButton.readValue()
            resetHeadingButton.readValue()

            robot.drive.sniperMode = gamepad1.left_trigger >= 0.2

            if (resetHeadingButton.wasJustPressed()) {
                robot.drive.resetFieldCentric()
                gamepad1.rumble(1.0, 1.0, 300)
            }

            robot.drive.driveFieldCentric(
                gamepad1.left_stick_x,
                -gamepad1.left_stick_y,
                gamepad1.right_stick_x
            )

            val armGamepad = -gamepad2.left_stick_y

            val armInput =
            if (
                (armGamepad < 0.0 && robot.arm.position > Math.toRadians(90.0)) ||
                (armGamepad > 0.0 && robot.arm.position < Math.toRadians(90.0))
            ) {
                armGamepad / 2.0
            } else {
                -armGamepad
            }

            robot.arm.power = armInput

            if (liftLimitButton.wasJustPressed()) {
                robot.lift.toggleLimits()
                gamepad1.rumble(1.0, 1.0, 300)
                gamepad2.rumble(1.0, 1.0, 300)
            }

            robot.lift.power = -gamepad2.right_stick_y
            robot.claw.position = gamepad2.right_trigger

            telemetry.update()
        }
    }
}