package org.firstinspires.ftc.teamcode.evenimente.liga_wonder

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.CONFIG
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.LEFT_ARM_SERVO_RANGE
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.LEFT_CLAW_SERVO_RANGE
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.RIGHT_ARM_SERVO_RANGE
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.RIGHT_CLAW_SERVO_RANGE
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.hardware.ServoEx

@Disabled
@TeleOp
class ClawTest : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val right = ServoEx(hardwareMap, CONFIG.RIGHT_CLAW, RIGHT_CLAW_SERVO_RANGE)
        val left = ServoEx(hardwareMap, CONFIG.LEFT_CLAW, LEFT_CLAW_SERVO_RANGE, direction = Servo.Direction.REVERSE)

        var position = 0.5

        var previousTime = time
        var deltaTime = 1.0

        while (opModeInInit()) {
            right.position = position
            left.position = position

            deltaTime = (time - previousTime) * 60.0
            previousTime = time
        }

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                position += 0.005 * deltaTime
            }

            if (gamepad1.dpad_down) {
                position -= 0.005 * deltaTime
            }

            right.position = position
            left.position = position

            telemetry.addData("position", position)
            telemetry.addData("deltaTime", deltaTime)
            telemetry.update()

            deltaTime = (time - previousTime) * 60.0
            previousTime = time
        }

    }
}