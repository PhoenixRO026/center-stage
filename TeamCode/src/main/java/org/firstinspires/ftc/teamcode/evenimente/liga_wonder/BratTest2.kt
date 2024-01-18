package org.firstinspires.ftc.teamcode.evenimente.liga_wonder

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.ARM_RAMP_POS
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.CONFIG
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.LEFT_ARM_SERVO_RANGE
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.RIGHT_ARM_SERVO_RANGE
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.hardware.ServoEx
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.systems.Arm

@Disabled
@TeleOp
class BratTest2 : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val arm = Arm(hardwareMap, telemetry, ARM_RAMP_POS..1.0)

        var position = 0.5

        var previousTime = time
        var deltaTime = 1.0

        while (opModeInInit()) {
            arm.position = position

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

            arm.position = position

            telemetry.addData("position", position)
            telemetry.addData("deltaTime", deltaTime)
            telemetry.update()

            deltaTime = (time - previousTime) * 60.0
            previousTime = time
        }

    }
}