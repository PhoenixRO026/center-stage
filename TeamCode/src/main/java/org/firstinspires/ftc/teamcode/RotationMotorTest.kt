package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos

private const val motorResolution = ((((1.0+(46.0/11.0))) * (1.0+(46.0/11.0))) * 28.0)

private fun Int.toPosition() : Double {
    return -this.toRadians() + Math.toRadians(215.0)
}

private fun Int.toRadians() : Double {
    return this / motorResolution * (2.0 * PI)
}

@TeleOp
class RotationMotorTest : LinearOpMode() {
    lateinit var motor: DcMotorEx

    override fun runOpMode() {
        motor = hardwareMap.get(DcMotorEx::class.java, "motor")
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        waitForStart()

        while (opModeIsActive()) {
            val input = gamepad1.left_stick_x
            val power = -cos(motor.currentPosition.toPosition()) * 0.2 + input * 0.8
            motor.power = power

            telemetry.addData("power", power)
            telemetry.addData("ticks", motor.currentPosition)
            telemetry.addData("position", Math.toDegrees(motor.currentPosition.toPosition()))
            telemetry.addData("motor res", motorResolution)
            telemetry.update()
        }
    }

}

