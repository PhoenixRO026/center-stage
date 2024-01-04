package org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class MotorEx(
    hardwareMap: HardwareMap,
    deviceName: String,
    private val telemetry: Telemetry? = null,
    direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    zeroPowerBehavior: ZeroPowerBehavior = ZeroPowerBehavior.BRAKE
) {
    private val motor = hardwareMap.get(DcMotorEx::class.java, deviceName)

    var targetPosition: Int = 0
        set(value) {
            if (field != value) {
                motor.targetPosition = value
            }
            field = value
        }

    var mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        set(value) {
            if (field != value) {
                motor.mode = value
            }
            field = value
        }

    var power: Double = 0.0
        set(value) {
            val scaledValue = value.coerceIn(-1.0, 1.0)
            if (scaledValue != field) {
                motor.power = scaledValue
            }
            field = scaledValue
        }

    var disabled: Boolean = false
        set(value) {
            if (field != value) {
                if (value) {
                    motor.setMotorDisable()
                } else {
                    motor.setMotorEnable()
                }
            }
            field = value
        }

    val position: Int
        get() = motor.currentPosition

    init {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.zeroPowerBehavior = zeroPowerBehavior
        motor.direction = direction
    }
}