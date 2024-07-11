package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.lib.hardware.motor.SimpleMotor.Companion.simpleMotor

@Disabled
@TeleOp(group = "Debug")
class MotorTest : LinearOpMode() {
    override fun runOpMode() {
        val motor = hardwareMap.simpleMotor("motor")

        waitForStart()

        while (opModeIsActive()) {
            motor.power = (gamepad1.right_trigger - gamepad1.left_trigger).toDouble()
        }
    }
}