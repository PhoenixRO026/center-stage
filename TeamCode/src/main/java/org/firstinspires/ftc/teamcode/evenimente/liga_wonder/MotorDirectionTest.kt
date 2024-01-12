package org.firstinspires.ftc.teamcode.evenimente.liga_wonder

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.CONFIG
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.LEFT_BACK_DRIVE_DIRECTION
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.LEFT_FRONT_DRIVE_DIRECTION
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.RIGHT_BACK_DRIVE_DIRECTION
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.RIGHT_FRONT_DRIVE_DIRECTION

@TeleOp
class MotorDirectionTest: LinearOpMode() {
    override fun runOpMode() {
        val leftBack = hardwareMap.get(DcMotorEx::class.java, CONFIG.LEFT_BACK_DRIVE)
        val leftFront = hardwareMap.get(DcMotorEx::class.java, CONFIG.LEFT_FRONT_DRIVE)
        val rightBack = hardwareMap.get(DcMotorEx::class.java, CONFIG.RIGHT_BACK_DRIVE)
        val rightFront = hardwareMap.get(DcMotorEx::class.java, CONFIG.RIGHT_FRONT_DRIVE)

        leftBack.direction = LEFT_BACK_DRIVE_DIRECTION
        leftFront.direction = LEFT_FRONT_DRIVE_DIRECTION
        rightBack.direction = RIGHT_BACK_DRIVE_DIRECTION
        rightFront.direction = RIGHT_FRONT_DRIVE_DIRECTION

        val motors = listOf(leftBack, leftFront, rightBack, rightFront)

        motors.forEach {
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            it.power = 0.0
        }

        waitForStart()

        while (opModeIsActive()) {
            rightFront.power = gamepad1.y.toDouble()
            leftFront.power = gamepad1.x.toDouble()
            rightBack.power = gamepad1.b.toDouble()
            leftBack.power = gamepad1.a.toDouble()
        }
    }

    fun Boolean.toDouble() = if (this) 1.0 else 0.0
}