package org.firstinspires.ftc.teamcode.evenimente.kickoff.robot

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.abs
import kotlin.math.sign

class Lift(
    hardwareMap: HardwareMap,
    private val telemetry: Telemetry? = null
) {
    private val motor = hardwareMap.get(DcMotorEx::class.java, LIFT_ID)
    private val liftBottom = 0
    private val liftTop = 1600
    private val holdPower = 0.1
    private enum class LiftState {
        ADVANCING,
        BRAKING,
        HOLDING,
        NO_PID
    }
    private var state = LiftState.HOLDING
    private val minBrakeVelocity = 150
    var power: Number = 0.0
        set(value) {
            val clampedVal = value.toDouble().coerceIn(-1.0, 1.0)
            if (clampedVal == field.toDouble())
                return
            if (motor.mode == DcMotor.RunMode.RUN_TO_POSITION && sign(clampedVal) != sign(field.toDouble())) {
                if (clampedVal == 0.0) state = LiftState.BRAKING
                else {
                    state = LiftState.ADVANCING
                    motor.targetPosition = if (clampedVal > 0.0) liftTop else liftBottom
                }
            }
            motor.power = clampedVal
            field = clampedVal
        }
    var limitsDisabled = false
        set(value) {
            if (value == field)
                return
            if (value) {
                motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                state = LiftState.NO_PID
                motor.power = power.toDouble()
            } else {
                state = LiftState.BRAKING
                motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                motor.targetPosition = 0
                motor.power = 0.0
                motor.mode = DcMotor.RunMode.RUN_TO_POSITION
            }
            field = value
        }

    init {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.direction = DcMotorSimple.Direction.REVERSE
        motor.targetPosition = 0
        motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        motor.power = holdPower
    }

    fun update() {
        if (motor.mode == DcMotor.RunMode.RUN_TO_POSITION && motor.power == 0.0 && abs(motor.velocity) <= minBrakeVelocity) {
            state = LiftState.HOLDING
            motor.targetPosition = motor.currentPosition
            motor.power = holdPower
        }
        telemetry?.addData("Lift power", power.toDouble())
        telemetry?.addData("Lift position", motor.currentPosition)
        telemetry?.addData("Lift target position", motor.targetPosition)
        telemetry?.addData("Lift velocity", motor.velocity)
        telemetry?.addData("Lift state", state)
        telemetry?.addData("Lift mode", motor.mode)
        telemetry?.addData("Lift motor power", motor.power)
        telemetry?.addData("Limits enabled", !limitsDisabled)
    }
}