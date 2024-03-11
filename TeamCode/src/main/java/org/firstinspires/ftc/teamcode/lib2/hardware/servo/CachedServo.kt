package org.firstinspires.ftc.teamcode.lib2.hardware.servo

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo.Direction
import kotlin.math.abs

class CachedServo @JvmOverloads constructor(
    @JvmField val innerServo: BasicServo,
    @JvmField val cachingThreshold: Double = defaultCachingThreshold
) : ServoEx {
    companion object {
        const val defaultCachingThreshold = 0.01

        fun HardwareMap.cachedServo(
            deviceName: String,
            direction: Direction = Direction.FORWARD,
            cachingThreshold: Double = defaultCachingThreshold
        ) = CachedServo(this, deviceName, direction, cachingThreshold)
    }

    @JvmOverloads constructor(
        hardwareMap: HardwareMap,
        deviceName: String,
        direction: Direction = Direction.FORWARD,
        cachingThreshold: Double = defaultCachingThreshold
    ) : this(BasicServo(hardwareMap, deviceName, direction), cachingThreshold)

    private var hasPositionBeenSet = false

    private var innerPosition: Double by innerServo::position

    override var position: Double = innerPosition
        set(value) {
            val newValue = value.coerceIn(0.0, 1.0)
            if (shouldChangePosition(newValue)) {
                hasPositionBeenSet = true
                innerPosition = newValue
            }
            field = newValue
        }

    fun shouldChangePosition(pos: Double): Boolean {
        val newPosition = pos.coerceIn(0.0, 1.0)
        val overChangeThreshold = abs(newPosition - innerPosition) >= cachingThreshold
        val targetingMaxPosition = (newPosition >= 1.0 && innerPosition < 1.0) || (newPosition <= 0.0 && innerPosition > 0.0)
        return overChangeThreshold || targetingMaxPosition || !hasPositionBeenSet
    }

    fun setPositionResult(position: Double): Boolean {
        val pos = position.coerceIn(0.0, 1.0)
        val result = shouldChangePosition(pos)
        this.position = pos
        return result
    }

    fun forceUpdate() {
        innerPosition = position
    }

    fun rangedSpeed(speed: Double = SpeedServo.defaultSpeed, range: ClosedRange<Double> = 0.0..1.0) = SpeedServo(this, speed).rangedSpeed(range)
}