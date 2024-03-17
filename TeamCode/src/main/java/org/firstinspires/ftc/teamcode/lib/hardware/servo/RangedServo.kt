package org.firstinspires.ftc.teamcode.lib.hardware.servo

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

open class RangedServo @JvmOverloads constructor(
    servo: Servo,
    direction: Servo.Direction = Servo.Direction.FORWARD,
    @JvmField val range: ClosedRange<Double> = 0.0..1.0
): SimpleServo(servo, direction) {
    companion object {
        fun HardwareMap.rangedServo(
            deviceName: String,
            direction: Servo.Direction = Servo.Direction.FORWARD,
            range: ClosedRange<Double> = 0.0..1.0
        ) = RangedServo(this, deviceName, direction, range)
    }

    @JvmOverloads constructor(
        hardwareMap: HardwareMap,
        deviceName: String,
        direction: Servo.Direction = Servo.Direction.FORWARD,
        range: ClosedRange<Double> = 0.0..1.0
    ) : this(hardwareMap.get(Servo::class.java, deviceName), direction, range)

    var unscaledPosition
        get() = super.position
        set(value) {
            super.position = value
        }

    override var position: Double
        get() = unscaledPosition.reverseScale(range).coerceIn(0.0, 1.0)
        set(value) {
            unscaledPosition = value.coerceIn(0.0, 1.0).scaleTo(range)
        }
}