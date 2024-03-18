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
            kP = 0.0,
            kI = 0.0,
            kD = 0.0
        )
        @JvmField var toleranceTicks = 10
    }

    private val leftMotor = hardwareMap.simpleMotor("leftLift", DcMotorSimple.Direction.REVERSE)
    private val rightMotor = hardwareMap.simpleMotor(
        deviceName = "rightLift",
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

    val positionTicks by rightMotor::positionTicks

    var targetPositionTicks = 0

    val isBusy get() = mode == MODE.TARGET && abs(targetPositionTicks - positionTicks) > LiftConfig.toleranceTicks

    fun update() {
        val feedback = LiftConfig.controller.calculate(positionTicks.toDouble(), targetPositionTicks.toDouble())

        if (mode == MODE.TARGET) {
            rightMotor.power = feedback
        }
    }
}