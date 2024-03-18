package org.firstinspires.ftc.teamcode.lib.hardware.servo

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.Servo.Direction
import org.firstinspires.ftc.teamcode.lib.hardware.servo.SpeedServo.Companion.defaultSpeed
import org.firstinspires.ftc.teamcode.lib.units.DeltaTime
import kotlin.math.abs
import kotlin.math.sign

open class SpeedCachedServo(
    servo: Servo,
    direction: Direction = Direction.FORWARD,
    cachingThreshold: Double = defaultCachingThreshold,
    @JvmField val speed: Double = defaultSpeed,
    private val coupledServos: List<SimpleServo> = emptyList()
) : CachedServo(servo, direction, cachingThreshold) {

    companion object {
        fun HardwareMap.speedCachedServo(
            deviceName: String,
            direction: Direction = Direction.FORWARD,
            cachingThreshold: Double = defaultCachingThreshold,
            speed: Double = defaultSpeed,
            coupledServos: List<SimpleServo> = emptyList()
        ) = SpeedCachedServo(this, deviceName, direction, cachingThreshold, speed, coupledServos)
    }

    @JvmOverloads constructor(
        hardwareMap: HardwareMap,
        deviceName: String,
        direction: Direction = Direction.FORWARD,
        cachingThreshold: Double = defaultCachingThreshold,
        speed: Double = defaultSpeed,
        coupledServos: List<SimpleServo> = emptyList()
    ) : this(hardwareMap.get(Servo::class.java, deviceName), direction, cachingThreshold, speed, coupledServos)

    private val deltaTime = DeltaTime()

    private var innerPosition: Double
        get() = super.position
        set(value) {
            super.position = value
            updateCoupledServos()
            targetPosition = value.coerceIn(0.0, 1.0)
        }

    override var position: Double by ::innerPosition

    private var innerTargetPosition: Double = super.position

    open var targetPosition: Double by ::innerTargetPosition

    val isBusy: Boolean get() = innerTargetPosition != innerPosition

    override fun setPositionResult(position: Double): Boolean {
        val pos = position.coerceIn(0.0, 1.0)
        val result = shouldChangePosition(pos)
        this.innerPosition = pos
        return result
    }

    fun updateCoupledServos() {
        coupledServos.forEach {
            it.position = cachedPosition
        }
    }

    fun update() {
        val step = speed * deltaTime.calculateDeltaTime().s
        val error = innerTargetPosition - innerPosition
        if (error == 0.0) return
        if (abs(error) < step) {
            super.position += error
            forceUpdate()
        } else {
            super.position += error.sign * step
        }
        updateCoupledServos()
    }
}