package org.firstinspires.ftc.teamcode.kickoff

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.kickoff.PIDFController.PIDCoefficients
import kotlin.math.PI
import kotlin.math.cos

private const val motorResolution = ((((1.0+(46.0/11.0))) * (1.0+(46.0/11.0))) * 28.0)

//private const val motorMaxSpeedRadSec = 223.0 / 60.0 / (2.0 * PI)

private fun Int.toPosition() : Double {
    return -this.toRadians() + Math.toRadians(215.0)
}

private fun Int.toRadians() : Double {
    return this / motorResolution * (2.0 * PI)
}

@Disabled
@TeleOp
class PIDArm : LinearOpMode() {

    data class PIDCoeffs(
        @JvmField var kP: Double = 0.0,
        @JvmField var kI: Double = 0.0,
        @JvmField var kD: Double = 0.0
    )

    data class FDCoeffs(
        @JvmField var kV: Double = 0.0,
        @JvmField var kA: Double = 0.0,
        @JvmField var kStatic: Double = 0.0
    )

    @Config
    data object Consts {
        @JvmField var PID = PIDCoeffs()
        @JvmField var FD = FDCoeffs()
        @JvmField var kG = 0.0
    }

    override fun runOpMode() {
        val motor : DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "motor")
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        val coeffs = PIDCoefficients(Consts.PID.kP, Consts.PID.kI, Consts.PID.kD)

        val controller = PIDFController(coeffs, Consts.FD.kV, Consts.FD.kA, Consts.FD.kStatic) { _, _ -> -cos(motor.currentPosition.toPosition()) * Consts.kG }

        controller.setInputBounds(0.0, 2 * PI)

        waitForStart()

        while (opModeIsActive()) {
            controller.setCoeffs(PIDCoefficients(Consts.PID.kP, Consts.PID.kI, Consts.PID.kD), Consts.FD.kV, Consts.FD.kA, Consts.FD.kStatic) { _, _ -> -cos(motor.currentPosition.toPosition()) * Consts.kG }
            controller.targetVelocity = gamepad1.left_stick_x.toDouble()
            val power = controller.update(motor.currentPosition.toPosition())
            motor.power = power

            telemetry.addData("power", power)
            telemetry.addData("ticks", motor.currentPosition)
            telemetry.addData("position", motor.currentPosition.toPosition())
            telemetry.update()
        }
    }
}