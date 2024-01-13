package org.firstinspires.ftc.teamcode.evenimente.alba

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.evenimente.alba.robot.CONFIG_ALBA
import org.firstinspires.ftc.teamcode.evenimente.alba.robot.INTAKE_RANGE
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.hardware.ServoEx
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.systems.Intake

@TeleOp
class IntakeUpTest : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val servo = ServoEx(hardwareMap, CONFIG_ALBA.INTAKE_SERVO, 0.0..1.0)
        val motor = Intake(hardwareMap)

        var position = 0.5

        var previousTime = time
        var deltaTime = 1.0

        while (opModeInInit()) {
            servo.position = position

            deltaTime = (time - previousTime) * 60.0
            previousTime = time
        }

        while (opModeIsActive()) {
            motor.power = (gamepad1.right_trigger - gamepad1.left_trigger).toDouble()

            if (gamepad1.dpad_up) {
                position += 0.005 * deltaTime
            }

            if (gamepad1.dpad_down) {
                position -= 0.005 * deltaTime
            }

            servo.position = position

            telemetry.addData("position", position)
            telemetry.addData("deltaTime", deltaTime)
            telemetry.update()

            deltaTime = (time - previousTime) * 60.0
            previousTime = time
        }

    }
}