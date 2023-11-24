package org.firstinspires.ftc.teamcode.evenimente.kickoff.robot

import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.PI

class Arm(
    hardwareMap: HardwareMap,
    private val telemetry: Telemetry? = null
) {
    private val motor = hardwareMap.get(DcMotorEx::class.java, ARM_ID)
    private val motorResolution = ((((1.0+(46.0/17.0))) * (1.0+(46.0/17.0))) * (1.0+(46.0/17.0)) * 28.0)
    private val motorMaxSpeedRadSec = 117.0 / 60.0 / (2.0 * PI)
    private val startPos = Math.toRadians(200.0)
    private val feedforward = ArmFeedforward(0.0, -0.35, 1.5, 0.0)
    var power : Number = 0.0
        set(value) {
            field = value.toDouble().coerceIn(-1.0, 1.0)
        }
    val position : Double
        get() {
            return motor.currentPosition.toPosition()
        }
    val velocity : Double
        get() {
            return motor.velocity.toRadians()
        }

    init {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.direction = DcMotorSimple.Direction.REVERSE
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.power = 0.0
    }

    fun update() {
        motor.power = feedforward.calculate(position, power.toDouble() * motorMaxSpeedRadSec)

        telemetry?.addData("ARM MOTOR POWER", motor.power)
        telemetry?.addData("ARM POSITION DEGREES", position.toDegrees())
        telemetry?.addData("ARM CURRENT VELOCITY DEGREES", velocity.toDegrees())
        telemetry?.addData("ARM TARGET VELOCITY DEGREES", (power.toDouble() * motorMaxSpeedRadSec).toDegrees())
    }

    private fun Int.toRadians() : Double {
        return this.toDouble().toRadians()
    }

    private fun Double.toRadians() : Double {
        return this / motorResolution * (2.0 * PI)
    }

    private fun Int.toPosition() : Double {
        return -this.toRadians() + startPos
    }

    private fun Double.toDegrees() : Double {
        return Math.toDegrees(this)
    }
}