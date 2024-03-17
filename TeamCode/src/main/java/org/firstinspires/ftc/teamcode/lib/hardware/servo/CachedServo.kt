package org.firstinspires.ftc.teamcode.lib.hardware.servo

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.Servo.Direction
import kotlin.math.abs

open class CachedServo @JvmOverloads constructor(
    servo: Servo,
    direction: Direction = Direction.FORWARD,
    @JvmField val cachingThreshold: Double = defaultCachingThreshold
) : SimpleServo(servo, direction) {
    companion object {
        fun HardwareMap.cachedServo(
            deviceName: String,
            direction: Direction = Direction.FORWARD,
            cachingThreshold: Double = defaultCachingThreshold
        ) = CachedServo(this, deviceName, direction, cachingThreshold)

        const val defaultCachingThreshold = 0.01
    }

    @JvmOverloads constructor(
        hardwareMap: HardwareMap,
        deviceName: String,
        direction: Direction = Direction.FORWARD,
        cachingThreshold: Double = defaultCachingThreshold
    ) : this(hardwareMap.get(Servo::class.java, deviceName), direction, cachingThreshold)

    open val cachedPosition get() = super.position

    override var position: Double = super.position
        set(value) {
            val newValue = value.coerceIn(0.0, 1.0)
            if (shouldChangePosition(newValue)) {
                super.position = newValue
            }
            field = newValue
        }

    fun shouldChangePosition(pos: Double): Boolean {
        val newPosition = pos.coerceIn(0.0, 1.0)
        val overChangeThreshold = abs(newPosition - super.position) >= cachingThreshold
        val targetingMaxPosition = (newPosition >= 1.0 && super.position < 1.0) || (newPosition <= 0.0 && super.position > 0.0)
        return overChangeThreshold || targetingMaxPosition || !hasPositionBeenSet
    }

    fun forceUpdate() {
        super.position = position
    }

    open fun setPositionResult(position: Double): Boolean {
        val pos = position.coerceIn(0.0, 1.0)
        val result = shouldChangePosition(pos)
        this.position = pos
        return result
    }
}