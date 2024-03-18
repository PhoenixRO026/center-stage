package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.lib.hardware.motor.SimpleMotor.Companion.simpleMotor

@Disabled
@TeleOp
class LiftTest : LinearOpMode(){
    override fun runOpMode() {
        val leftMotor = hardwareMap.simpleMotor("leftLift", DcMotorSimple.Direction.REVERSE)
        val rightMotor = hardwareMap.simpleMotor("rightLift")

        waitForStart()

        while (opModeIsActive()) {
            val power = -gamepad1.right_stick_y.toDouble()

            leftMotor.power = power
            rightMotor.power = power

            telemetry.addData("power", power)
            telemetry.update()
        }
    }
}