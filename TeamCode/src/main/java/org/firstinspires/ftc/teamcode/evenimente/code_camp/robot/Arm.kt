package org.firstinspires.ftc.teamcode.evenimente.code_camp.robot

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.evenimente.code_camp.robot.feedforward.ArmFeedforward
import org.firstinspires.ftc.teamcode.evenimente.kickoff.robot.ARM_ID
import kotlin.math.PI

class Arm(
    hardwareMap: HardwareMap,
    private val telemetry: Telemetry? = null
) {
    @Config
    data object ArmConfig {
        @JvmField var ks: Double = 0.26
        @JvmField var kcos: Double = -0.29
        @JvmField var kv: Double = 0.4
        @JvmField var ka: Double = 0.0
        @JvmField var MOVING_TRESHHOLD_RAD_SEC: Double = 0.05
        @JvmField var DOWN_POWER: Double = -0.0000000001
    }
    private val motor = hardwareMap.get(DcMotorEx::class.java, ARM_ID)
    private val battery = hardwareMap.voltageSensor.iterator().next()
    private val voltage: Double
        get() = battery.voltage
    private val gearRatio = (68.0 / 13.0) * (76.0 / 21.0) * (84.0 / 29.0) //from rev website
    private val motorResolution = gearRatio * 28.0
    private val motorMaxSpeedRadSec = (6000.0 / gearRatio).rpmToRadPerSec()
    private val startPos = 195.0.degreesToRadians()
    private val feedforward = ArmFeedforward(
        ArmConfig.ks,
        ArmConfig.kcos,
        ArmConfig.kv,
        ArmConfig.ka
    )
    var power : Number = 0.0
        set(value) {
            field = value.toDouble().coerceIn(-1.0, 1.0)
        }
    val position : Double
        get() {
            return motor.currentPosition.ticksToPositionRad()
        }
    val velocity : Double
        get() {
            return motor.velocity.ticksToRadians()
        }

    init {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.direction = DcMotorSimple.Direction.REVERSE
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.power = 0.0
    }

    fun update() {
        val kPower = maxOf(power.toDouble(),
            org.firstinspires.ftc.teamcode.evenimente.code_camp.robot.Arm.ArmConfig.DOWN_POWER
        )
        val ks = if (velocity < org.firstinspires.ftc.teamcode.evenimente.code_camp.robot.Arm.ArmConfig.MOVING_TRESHHOLD_RAD_SEC) org.firstinspires.ftc.teamcode.evenimente.code_camp.robot.Arm.ArmConfig.ks else 0.0
        feedforward.updateCoeficients(ks,
            org.firstinspires.ftc.teamcode.evenimente.code_camp.robot.Arm.ArmConfig.kcos,
            org.firstinspires.ftc.teamcode.evenimente.code_camp.robot.Arm.ArmConfig.kv,
            org.firstinspires.ftc.teamcode.evenimente.code_camp.robot.Arm.ArmConfig.ka
        )
        motor.power = feedforward.calculate(position, kPower * motorMaxSpeedRadSec, voltage)

        telemetry?.addData("ARM VOLTAGE", voltage)
        telemetry?.addData("ARM MOTOR POWER", motor.power)
        telemetry?.addData("ARM POSITION DEGREES", position.toDegrees())
        telemetry?.addData("ARM CURRENT VELOCITY RADS PER SEC", velocity)
        telemetry?.addData("ARM TARGET VELOCITY RADS PER SEC", power.toDouble() * motorMaxSpeedRadSec)
    }

    private fun Int.ticksToRadians() : Double {
        return this.toDouble().ticksToRadians()
    }

    private fun Double.ticksToRadians(): Double {
        return this / motorResolution * (2.0 * PI) //1 rev to rads
    }

    private fun Int.ticksToPositionRad(): Double {
        return -this.ticksToRadians() + startPos
    }

    private fun Double.toDegrees(): Double {
        return Math.toDegrees(this)
    }

    private fun Double.degreesToRadians(): Double {
        return Math.toRadians(this)
    }

    private fun Double.rpmToRadPerSec(): Double {
        return this / 60.0 /*seconds*/ / (2.0 * PI) /*rad*/
    }
}