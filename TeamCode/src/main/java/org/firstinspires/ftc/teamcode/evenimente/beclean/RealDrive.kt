package org.firstinspires.ftc.teamcode.evenimente.beclean

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.ARM_SCORE_POS
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.CLAW_RAMP_POS
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.CLAW_SCORE_POS
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.FINGERS_INTAKE_OPEN_POS
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.Robot2

@TeleOp
class RealDrive : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val robot = Robot2(hardwareMap, telemetry)

        val intakeToggle = ToggleButtonReader {
            gamepad2.left_stick_button
        }

        waitForStart()

        while (isStarted && !isStopRequested) {
            robot.update()
            intakeToggle.readValue()

            if (gamepad1.y) {
                robot.drive.resetFieldCentric()
            }

            robot.drive.sniperMode = gamepad1.left_trigger >= 0.2

            robot.drive.driveFieldCentric(
                -gamepad1.left_stick_y.toDouble(),
                gamepad1.left_stick_x.toDouble(),
                gamepad1.right_stick_x.toDouble()
            )

            robot.lift.power = -gamepad2.right_stick_y.toDouble()

            if (gamepad2.dpad_up) {
                robot.arm.position += 0.0075 * robot.deltaTime
            }

            if (gamepad2.dpad_down) {
                robot.arm.position -= 0.0075 * robot.deltaTime
            }

            if (gamepad2.y) {
                robot.claw.angle += 0.0075 * robot.deltaTime
            }

            if (gamepad2.a) {
                robot.claw.angle -= 0.0075 * robot.deltaTime
            }

            if (gamepad2.left_bumper) {
                robot.arm.position = 0.0
                robot.claw.angle = CLAW_RAMP_POS
            }

            if (gamepad2.right_bumper) {
                robot.arm.position = ARM_SCORE_POS
                robot.claw.angle = CLAW_SCORE_POS
            }

            if (gamepad2.x) {
                robot.claw.leftFingerPos = FINGERS_INTAKE_OPEN_POS
                robot.claw.rightFingerPos = FINGERS_INTAKE_OPEN_POS
            } else {
                robot.claw.leftFingerPos = 1.0 - gamepad2.right_trigger.toDouble()
                robot.claw.rightFingerPos = 1.0 - gamepad2.left_trigger.toDouble()
            }

            robot.intake.power = -gamepad2.left_stick_y.toDouble()

            if (gamepad2.back) {
                robot.lift.hang()
            }

            if (gamepad2.start) {
                robot.lift.unhang()
            }

            if (gamepad2.b) {
                robot.plane.launch()
            }

            if (intakeToggle.state) {
                robot.intake.angle = 1.0
            } else {
                robot.intake.angle = 0.0
            }

            telemetry.addData("arm pos", robot.arm.position)
            telemetry.addData("claw angle", robot.claw.angle)
            telemetry.addData("left lift pos", robot.lift.leftPosition)
            telemetry.addData("heading", robot.drive.heading)
            telemetry.update()
        }
    }
}