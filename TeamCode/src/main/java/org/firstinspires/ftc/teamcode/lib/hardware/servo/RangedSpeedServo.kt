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
    private val coupledServos: List<SimpleServo> = emptyList()
) : SpeedServo(servo, direction, speed) {
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
            coupledServos.forEach {
                it.position = position
            }
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

    override fun update() {
        val error = unscaledTargetPosition - unscaledPosition
        if (error == 0.0) return
        val step = speed * deltaTime.calculateDeltaTime().s
        super.position += if (abs(error) < step) {
            error
        } else {
            error.sign * step
        }
        coupledServos.forEach {
            it.position = position
        }
    }
}