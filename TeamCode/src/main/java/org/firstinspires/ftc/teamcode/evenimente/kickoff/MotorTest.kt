package org.firstinspires.ftc.teamcode.evenimente.kickoff

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx

@Disabled
//@TeleOp
class MotorTest : LinearOpMode() {
    override fun runOpMode() {
        val motor: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "motor")
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        waitForStart()
        while (opModeIsActive()) {
            val power = gamepad1.left_stick_y.toDouble()
            motor.power = power
            telemetry.addData("power", power)
            telemetry.update()
        }
    }
}