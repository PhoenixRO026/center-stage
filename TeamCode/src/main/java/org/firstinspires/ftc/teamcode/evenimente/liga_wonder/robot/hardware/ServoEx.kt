package org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.hardware

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.Telemetry

class ServoEx(
    hardwareMap: HardwareMap,
    deviceName: String,
    val range: ClosedRange<Double> = 0.0..1.0,
    private val telemetry: Telemetry? = null,
    direction: Servo.Direction = Servo.Direction.FORWARD
) {
    private val servo = hardwareMap.get(Servo::class.java, deviceName) as ServoImplEx

    var position: Double = 0.0
        set(value) {
            val scaledValue = value.coerceIn(0.0, 1.0).scaleTo(range)
            if (scaledValue != field) {
                servo.position = scaledValue
            }
            field = value.coerceIn(0.0, 1.0)
        }

    var disabled: Boolean = false
        set(value) {
            if (value != field) {
                if (value) {
                    servo.setPwmDisable()
                } else {
                    servo.setPwmEnable()
                }
            }
            field = value
        }

    init {
        servo.direction = direction
    }
}