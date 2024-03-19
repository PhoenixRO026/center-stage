package org.firstinspires.ftc.teamcode.systems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.lib.controller.PIDController
import org.firstinspires.ftc.teamcode.lib.hardware.motor.SimpleMotor.Companion.simpleMotor
import kotlin.math.abs

class Lift(
    hardwareMap: HardwareMap
) {
    companion object {
        fun HardwareMap.lift() = Lift(this)
    }

    @Config
    data object LiftConfig {
        @JvmField var controller = PIDController(
            kP = 0.01,
            kI = 0.005,
            kD = 0.001
        )
        @JvmField var toleranceTicks = 16
        @JvmField var kF = 0.16
    }

    private val leftMotor = hardwareMap.simpleMotor("leftLift")
    private val rightMotor = hardwareMap.simpleMotor(
        deviceName = "rightLift",
        direction = DcMotorSimple.Direction.REVERSE,
        coupledMotors = listOf(leftMotor)
    )

    enum class MODE {
        RAW,
        TARGET
    }

    var mode = MODE.RAW
        set(value) {
            if (value == MODE.TARGET) {
                LiftConfig.controller.resetIntegral()
            }
            field = value
        }

    var power = rightMotor.power
        set(value) {
            if (mode == MODE.RAW) {
                rightMotor.power = value
            }
            field = value
        }

    val positionTicks get() = -rightMotor.positionTicks

    var targetPositionTicks = 0

    val isBusy get() = mode == MODE.TARGET && abs(targetPositionTicks - positionTicks) > LiftConfig.toleranceTicks

    fun update() {
        val feedback = LiftConfig.controller.calculate(positionTicks.toDouble(), targetPositionTicks.toDouble()) + LiftConfig.kF

        if (mode == MODE.TARGET) {
            rightMotor.power = -feedback
        }
    }
}