package org.firstinspires.ftc.teamcode.lib.hardware.servo

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.lib.hardware.servo.SpeedServo.Companion.defaultSpeed

class RangedSpeedCachedServo(
    servo: Servo,
    direction: Servo.Direction = Servo.Direction.FORWARD,
    cachingThreshold: Double = defaultCachingThreshold,
    speed: Double = defaultSpeed,
    @JvmField val range: ClosedRange<Double> = 0.0..1.0
) : SpeedCachedServo(servo, direction, cachingThreshold, speed) {
    companion object {
        fun HardwareMap.rangedSpeedCachedServo(
            deviceName: String,
            direction: Servo.Direction = Servo.Direction.FORWARD,
            cachingThreshold: Double = defaultCachingThreshold,
            speed: Double = defaultSpeed,
            range: ClosedRange<Double> = 0.0..1.0
        ) = RangedSpeedCachedServo(this, deviceName, direction, cachingThreshold, speed, range)
    }

    @JvmOverloads constructor(
        hardwareMap: HardwareMap,
        deviceName: String,
        direction: Servo.Direction = Servo.Direction.FORWARD,
        cachingThreshold: Double = defaultCachingThreshold,
        speed: Double = defaultSpeed,
        range: ClosedRange<Double> = 0.0..1.0
    ) : this(hardwareMap.get(Servo::class.java, deviceName), direction, cachingThreshold, speed, range)
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

    fun setUnscaledPositionResult(position: Double): Boolean = super.setPositionResult(position)

    override fun setPositionResult(position: Double): Boolean = setUnscaledPositionResult(position.coerceIn(0.0, 1.0).scaleTo(range))
}