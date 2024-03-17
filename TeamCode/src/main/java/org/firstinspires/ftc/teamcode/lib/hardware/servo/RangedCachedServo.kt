package org.firstinspires.ftc.teamcode.lib.hardware.servo

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.Servo.Direction

class RangedCachedServo @JvmOverloads constructor(
    servo: Servo,
    direction: Direction = Direction.FORWARD,
    cachingThreshold: Double = defaultCachingThreshold,
    @JvmField val range: ClosedRange<Double> = 0.0..1.0
) : CachedServo(servo, direction, cachingThreshold) {
    companion object {
        fun HardwareMap.rangedCachedServo(
            deviceName: String,
            direction: Direction = Direction.FORWARD,
            cachingThreshold: Double = defaultCachingThreshold,
            range: ClosedRange<Double> = 0.0..1.0
        ) = RangedCachedServo(this, deviceName, direction, cachingThreshold, range)
    }

    @JvmOverloads constructor(
        hardwareMap: HardwareMap,
        deviceName: String,
        direction: Direction = Direction.FORWARD,
        cachingThreshold: Double = defaultCachingThreshold,
        range: ClosedRange<Double> = 0.0..1.0
    ) : this(hardwareMap.get(Servo::class.java, deviceName), direction, cachingThreshold, range)

    var unscaledPosition
        get() = super.position
        set(value) {
            super.position = value
        }

    fun setUnscaledPositionResult(position: Double): Boolean = super.setPositionResult(position)

    override fun setPositionResult(position: Double): Boolean = setUnscaledPositionResult(position.coerceIn(0.0, 1.0).scaleTo(range))

    override var position: Double
        get() = unscaledPosition.reverseScale(range).coerceIn(0.0, 1.0)
        set(value) {
            unscaledPosition = value.coerceIn(0.0, 1.0).scaleTo(range)
        }
}