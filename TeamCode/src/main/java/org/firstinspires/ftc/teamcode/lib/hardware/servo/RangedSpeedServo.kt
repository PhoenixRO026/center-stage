package org.firstinspires.ftc.teamcode.lib.hardware.servo

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.Servo.Direction
import kotlin.math.abs
import kotlin.math.sign

class RangedSpeedServo(
    servo: Servo,
    direction: Direction = Direction.FORWARD,
    speed: Double = defaultSpeed,
    @JvmField val range: ClosedRange<Double> = 0.0..1.0,
    coupledServos: List<SimpleServo> = emptyList()
) : SpeedServo(servo, direction, speed, coupledServos) {
    companion object {
        fun HardwareMap.rangedSpeedServo(
            deviceName: String,
            direction: Direction = Direction.FORWARD,
            speed: Double = defaultSpeed,
            range: ClosedRange<Double> = 0.0..1.0,
            coupledServos: List<SimpleServo> = emptyList()
        ) = RangedSpeedServo(this, deviceName, direction, speed, range, coupledServos)
    }

    @JvmOverloads constructor(
        hardwareMap: HardwareMap,
        deviceName: String,
        direction: Direction = Direction.FORWARD,
        speed: Double = defaultSpeed,
        range: ClosedRange<Double> = 0.0..1.0,
        coupledServos: List<SimpleServo> = emptyList()
    ) : this(hardwareMap.get(Servo::class.java, deviceName), direction, speed, range, coupledServos)

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

    var unscaledTargetPosition
        get() = super.targetPosition
        set(value) {
            super.targetPosition = value
        }

    override var targetPosition: Double
        get() = unscaledTargetPosition.reverseScale(range).coerceIn(0.0, 1.0)
        set(value) {
            unscaledTargetPosition = value.coerceIn(0.0, 1.0).scaleTo(range)
        }
}