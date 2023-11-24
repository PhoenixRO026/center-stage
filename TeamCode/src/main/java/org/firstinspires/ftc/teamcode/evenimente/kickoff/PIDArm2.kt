package org.firstinspires.ftc.teamcode.evenimente.kickoff

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import kotlin.math.PI

private const val motorResolution = ((((1.0+(46.0/17.0))) * (1.0+(46.0/17.0))) * (1.0+(46.0/17.0)) * 28.0)

private const val motorMaxSpeedRadSec = 117.0 / 60.0 / (2.0 * PI)

private fun Int.toPosition() : Double {
    return -this.toRadians() + Math.toRadians(215.0)
}

private fun Int.toRadians() : Double {
    return this / motorResolution * (2.0 * PI)
}

private fun Double.toRadians() : Double {
    return this / motorResolution * (2.0 * PI)
}

private fun Double.toDegrees() : Double {
    return Math.toDegrees(this)
}

@Disabled
@TeleOp
class PIDArm2 : LinearOpMode() {

    @Config
    object PIDArm2Config {
        @JvmField var kS = 0.0
        @JvmField var kCos = -0.35
        @JvmField var kV = 1.5
        @JvmField var kA = 0.0
        var oldS = 0.0
        var oldCos = 0.0
        var oldV = 0.0
        var oldA = 0.0

        fun hasChanged() : Boolean {
            if (kS != oldS || kCos != oldCos || kV != oldV || kA != oldA) {
                oldS = kS
                oldCos = kCos
                oldV = kV
                oldA = kA
                return true
            }
            return false
        }
    }
    override fun runOpMode() {
        val motor : DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "motor")
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        var feedforward = ArmFeedforward(PIDArm2Config.kS, PIDArm2Config.kCos, PIDArm2Config.kV, PIDArm2Config.kA)

        waitForStart()

        while (opModeIsActive()) {
            if (PIDArm2Config.hasChanged())
                feedforward = ArmFeedforward(PIDArm2Config.kS, PIDArm2Config.kCos, PIDArm2Config.kV, PIDArm2Config.kA)

            val target = gamepad1.left_stick_x * motorMaxSpeedRadSec

            val power = feedforward.calculate(motor.currentPosition.toPosition(), target)

            motor.power = power

            telemetry.addData("position", motor.currentPosition.toPosition())
            telemetry.addData("position degrees", motor.currentPosition.toPosition().toDegrees())
            telemetry.addData("power", power)
            telemetry.addData("targetVel", target)
            telemetry.addData("currentVel", motor.velocity.toRadians())
            telemetry.update()
        }
    }
}