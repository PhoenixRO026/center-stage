package org.firstinspires.ftc.teamcode.lib.hardware.servo

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.Servo.Direction
import org.firstinspires.ftc.teamcode.lib.units.DeltaTime
import kotlin.math.abs
import kotlin.math.sign

open class SpeedServo @JvmOverloads constructor(
    servo: Servo,
    direction: Direction = Direction.FORWARD,
    @JvmField val speed: Double = defaultSpeed,
    private val coupledServos: List<SimpleServo> = emptyList()
) : SimpleServo(servo, direction) {
    companion object {
        const val defaultSpeed = 0.1

        fun HardwareMap.speedServo(
            deviceName: String,
            direction: Direction = Direction.FORWARD,
            speed: Double = defaultSpeed,
            coupledServos: List<SimpleServo> = emptyList()
        ) = SpeedServo(this, deviceName, direction, speed, coupledServos)
    }

    @JvmOverloads constructor(
        hardwareMap: HardwareMap,
        deviceName: String,
        direction: Direction = Direction.FORWARD,
        speed: Double = defaultSpeed,
        coupledServos: List<SimpleServo> = emptyList()
    ) : this(hardwareMap.get(Servo::class.java, deviceName), direction, speed, coupledServos)

    protected val deltaTime = DeltaTime()

    override var position: Double
        get() = super.position
        set(value) {
            super.position = value
            coupledServos.forEach {
                it.position = super.position
            }
            targetPosition = value.coerceIn(0.0, 1.0)
        }

    open var targetPosition: Double = super.position

    open fun update() {
        val error = targetPosition - position
        if (error == 0.0) return
        val step = speed * deltaTime.calculateDeltaTime().s
        super.position += if (abs(error) < step) {
            error
        } else {
            error.sign * step
        }
        coupledServos.forEach {
            it.position = super.position
        }
    }
}