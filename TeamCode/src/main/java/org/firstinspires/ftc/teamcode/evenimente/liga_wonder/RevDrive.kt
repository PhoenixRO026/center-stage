package org.firstinspires.ftc.teamcode.evenimente.liga_wonder

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.ARM_SCORE_POS
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.CLAW_RAMP_POS
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.CLAW_SCORE_POS
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.FINGERS_INTAKE_OPEN_POS
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.Robot
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.systems.DetectionPipeline.DetectionPosition

@TeleOp
class RevDrive : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val robot = Robot(hardwareMap, telemetry)

        waitForStart()

        while (isStarted && !isStopRequested) {
            robot.update()

            if (gamepad1.y) {
                robot.drive.resetFieldCentric()
            }

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
                robot.claw.leftFingerPos = 1.0 - gamepad2.left_trigger.toDouble()
                robot.claw.rightFingerPos = 1.0 - gamepad2.right_trigger.toDouble()
            }

            robot.intake.power = -gamepad2.left_stick_y.toDouble()

            telemetry.addData("arm pos", robot.arm.position)
            telemetry.addData("claw angle", robot.claw.angle)
            telemetry.addData("left lift pos", robot.lift.leftPosition)
            telemetry.update()
        }
    }
}