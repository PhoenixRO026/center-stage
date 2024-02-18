package org.firstinspires.ftc.teamcode.lib.hardware

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.Servo.Direction
import com.qualcomm.robotcore.hardware.ServoControllerEx
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.lib.units.Angle
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.deg
import kotlin.math.abs
import kotlin.math.sign

@Suppress("unused", "MemberVisibilityCanBePrivate")
class ServoEx @JvmOverloads constructor(
    servo: Servo,
    positionRange: ClosedRange<Double> = 0.0..1.0,
    servoDirection: Direction = servo.direction,
    private val changeTreshold: Double = 0.0,
    private val maxAngle: Angle = 180.deg,
    var speed: Double = 0.1
) {
    companion object {
        const val MIN_PWM = 500.0
        const val MAX_PWM = 2500.0
        const val MAX_PWM_RANGE = MAX_PWM - MIN_PWM

        fun HardwareMap.servoEx(deviceName: String, direction: Direction? = null, range: ClosedRange<Double> = 0.0..1.0, changeTreshold: Double = 0.0, maxAngle: Angle = 180.deg, speed: Double = 0.1): ServoEx {
            val servo = get(Servo::class.java, deviceName)
            return ServoEx(
                servo = servo,
                positionRange = range,
                changeTreshold = changeTreshold,
                speed = speed,
                servoDirection = direction ?: servo.direction,
                maxAngle = maxAngle
            )
        }

        @JvmOverloads
        fun axonMax180(servo: Servo, direction: Direction = servo.direction, range: ClosedRange<Double> = 0.0..1.0, changeTreshold: Double = 0.0, speed: Double = 0.1) = ServoEx(
            servo = servo,
            maxAngle = 180.deg,
            positionRange = range,
            changeTreshold = changeTreshold,
            speed = speed,
            servoDirection = direction
        )

        fun HardwareMap.axonMax180(deviceName: String, direction: Direction? = null, range: ClosedRange<Double> = 0.0..1.0, changeTreshold: Double = 0.0, speed: Double = 0.1): ServoEx {
            val servo = get(Servo::class.java, deviceName)
            return axonMax180(
                servo = servo,
                range = range,
                changeTreshold = changeTreshold,
                speed = speed,
                direction = direction ?: servo.direction
            )
        }
        @JvmOverloads
        fun axonMax355(servo: Servo, direction: Direction = servo.direction, range: ClosedRange<Double> = 0.0..1.0, changeTreshold: Double = 0.0, speed: Double = 0.1) = ServoEx(
            servo = servo,
            maxAngle = 355.deg,
            positionRange = range,
            changeTreshold = changeTreshold,
            speed = speed,
            servoDirection = direction
        )

        fun HardwareMap.axonMax355(deviceName: String, direction: Direction? = null, range: ClosedRange<Double> = 0.0..1.0, changeTreshold: Double = 0.0, speed: Double = 0.1): ServoEx {
            val servo = get(Servo::class.java, deviceName)
            return axonMax355(
                servo = servo,
                range = range,
                changeTreshold = changeTreshold,
                speed = speed,
                direction = direction ?: servo.direction
            )
        }
    }

    private val positionRange: ClosedRange<Double> = (positionRange.start.coerceIn(0.0, positionRange.endInclusive.coerceAtMost(1.0)))..(positionRange.endInclusive.coerceIn(positionRange.start.coerceAtLeast(0.0), 1.0))

    private var cachedPosition: Double = 0.0
        set(value) {
            if (field != value) {
                internalServo.position = value
            }
            field = value
        }

    var unscaledTargetPosition: Double = 0.0

    val currentUnscaledPosition by ::cachedPosition

    fun shouldChangePosition(pos: Double): Boolean {
        val newPosition = pos.coerceIn(0.0, 1.0)
        val overChangeThreshold = abs(newPosition - cachedPosition) >= changeTreshold
        val targetingLimitPosition = (newPosition >= positionRange.endInclusive && cachedPosition < positionRange.endInclusive) || (newPosition <= positionRange.start && cachedPosition > positionRange.start)
        val targetingMaxPosition = (newPosition >= 1.0 && cachedPosition < 1.0) || (newPosition <= 0.0 && cachedPosition > 0.0)
        return overChangeThreshold || targetingLimitPosition || targetingMaxPosition
    }

    fun setUnscaledPositionResult(pos: Double): Boolean {
        val newPosition = pos.coerceIn(0.0, 1.0)
        return if (shouldChangePosition(newPosition)) {
            cachedPosition = newPosition
            true
        } else {
            false
        }
    }

    var unscaledPosition: Double = 0.0
        private set(value) {
            val newPosition = value.coerceIn(0.0, 1.0)
            if (shouldChangePosition(newPosition)) {
                cachedPosition = newPosition
            }
            field = newPosition
        }

    val internalServo: ServoImplEx = servo as ServoImplEx

    val controller: ServoControllerEx = internalServo.controller as ServoControllerEx

    val portNumber: Int = internalServo.portNumber

    val isBusy get() = unscaledPosition != unscaledTargetPosition

    fun setPositionResult(pos: Double) = setUnscaledPositionResult(pos.coerceIn(0.0, 1.0).scaleTo(positionRange))

    var position: Double
        get() = unscaledPosition.reverseScale(positionRange).coerceIn(0.0, 1.0)
        set(value) {
            val scaledPos = value.coerceIn(0.0, 1.0).scaleTo(positionRange)
            unscaledPosition = scaledPos
            unscaledTargetPosition = unscaledPosition
        }

    val currentPosition get() = currentUnscaledPosition.reverseScale(positionRange).coerceIn(0.0, 1.0)

    var targetPosition: Double
        get() = unscaledTargetPosition.reverseScale(positionRange).coerceIn(0.0, 1.0)
        set(value) {
            unscaledTargetPosition = value.coerceIn(0.0, 1.0).scaleTo(positionRange)
        }

    val servoRange get() = ((pwnRange.usPulseLower - MIN_PWM) / MAX_PWM_RANGE)..((pwnRange.usPulseUpper - MIN_PWM) / MAX_PWM_RANGE)

    fun setAngleResult(angle: Angle): Boolean = setUnscaledPositionResult((angle.coerceIn(0.deg, maxAngle) / maxAngle).reverseScale(servoRange))

    var angle: Angle
        get() = maxAngle * unscaledPosition.scaleTo(servoRange)
        set(value) {
            val scaledPos = (value.coerceIn(0.deg, maxAngle) / maxAngle).reverseScale(servoRange)
            unscaledPosition = scaledPos
            unscaledTargetPosition = unscaledPosition
        }

    var targetAngle: Angle
        get() = maxAngle * unscaledTargetPosition.scaleTo(servoRange)
        set(value) {
            unscaledTargetPosition = (value.coerceIn(0.deg, maxAngle) / maxAngle).reverseScale(servoRange)
        }

    var pwnRange: PwmRange = internalServo.pwmRange
        set(value) {
            if (value != field) {
                internalServo.pwmRange = value
            }
            field = value
        }

    var disabled: Boolean = !internalServo.isPwmEnabled
        set(value) {
            if (field != value) {
                if (value) {
                    internalServo.setPwmDisable()
                } else {
                    internalServo.setPwmEnable()
                }
            }
            field = value
        }

    var direction: Direction = internalServo.direction
        set(value) {
            if (field != value) {
                internalServo.direction = value
            }
            field = value
        }

    fun update(deltaTime: Time) {
        val error = unscaledTargetPosition - unscaledPosition
        if (error == 0.0) return
        val step = speed * deltaTime.s
        if (abs(error) < step) {
            unscaledPosition += error
            if (unscaledPosition != cachedPosition) {
                cachedPosition = unscaledTargetPosition
            }
        } else {
            unscaledPosition += sign(error) * step
        }
    }

    init {
        direction = servoDirection
    }
}