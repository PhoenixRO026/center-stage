package org.firstinspires.ftc.teamcode.evenimente.alba

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.evenimente.alba.robot.Arm2
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.ARM_RAMP_POS
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.ARM_SCORE_POS

//@TeleOp
class BratTest3 : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val arm = Arm2(hardwareMap, telemetry, ARM_RAMP_POS..1.0)

        var position = 0.5
        arm.position = position
        arm.goToTargetNow()

        var previousTime = time
        var deltaTime = 1.0

        while (opModeInInit()) {
            arm.position = position

            deltaTime = (time - previousTime) * 60.0
            previousTime = time

            arm.update(deltaTime)
        }

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                position += 0.005 * deltaTime
            }

            if (gamepad1.dpad_down) {
                position -= 0.005 * deltaTime
            }

            if (gamepad1.left_bumper) {
                position = 0.0
            }

            if (gamepad1.right_bumper) {
                position = ARM_SCORE_POS
            }

            arm.position = position

            arm.update(deltaTime)

            telemetry.addData("position", position)
            telemetry.addData("deltaTime", deltaTime)
            telemetry.update()

            deltaTime = (time - previousTime) * 60.0
            previousTime = time
        }

    }
}