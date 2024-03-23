package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorImplEx

@Disabled
@TeleOp
class MotorPortTest : LinearOpMode(){
    override fun runOpMode() {
        val motors = hardwareMap.getAll(DcMotor::class.java).map { it as DcMotorImplEx }.sortedBy { it.portNumber }

        waitForStart()

        while (opModeIsActive()) {
            if (motors.isNotEmpty()) {
                telemetry.addLine("press UP for port ${motors[0].portNumber}")
                if (gamepad1.dpad_up) {
                    motors[0].power = 1.0
                } else {
                    motors[0].power = 0.0
                }
            }

            if (motors.size > 1) {
                telemetry.addLine("press RIGHT for port ${motors[1].portNumber}")
                if (gamepad1.dpad_right) {
                    motors[1].power = 1.0
                } else {
                    motors[1].power = 0.0
                }
            }

            if (motors.size > 2) {
                telemetry.addLine("press DOWN for port ${motors[2].portNumber}")
                if (gamepad1.dpad_down) {
                    motors[2].power = 1.0
                } else {
                    motors[2].power = 0.0
                }
            }

            if (motors.size > 3) {
                telemetry.addLine("press LEFT for port ${motors[3].portNumber}")
                if (gamepad1.dpad_left) {
                    motors[3].power = 1.0
                } else {
                    motors[3].power = 0.0
                }
            }

            telemetry.update()
        }
    }
}