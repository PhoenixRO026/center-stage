package org.firstinspires.ftc.teamcode.stc.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

@TeleOp
class MotorTest: LinearOpMode() {
    override fun runOpMode() {
        val leftBack = hardwareMap.get(DcMotor::class.java, "leftBack")
        val leftFront = hardwareMap.get(DcMotor::class.java, "leftFront")
        val rightBack = hardwareMap.get(DcMotor::class.java, "rightBack")
        val rightFront = hardwareMap.get(DcMotor::class.java, "rightFront")

        leftBack.direction = DcMotorSimple.Direction.REVERSE
        leftFront.direction = DcMotorSimple.Direction.REVERSE

        waitForStart()

        while (opModeIsActive()) {
            rightFront.power = if (gamepad1.y) 1.0 else 0.0
            rightBack.power = if (gamepad1.b) 1.0 else 0.0
            leftFront.power = if (gamepad1.x) 1.0 else 0.0
            leftBack.power = if (gamepad1.a) 1.0 else 0.0
        }
    }
}