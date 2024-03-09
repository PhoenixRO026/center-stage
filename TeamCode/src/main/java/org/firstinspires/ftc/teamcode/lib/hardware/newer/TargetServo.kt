package org.firstinspires.ftc.teamcode.lib.hardware.newer

import org.firstinspires.ftc.teamcode.lib.hardware.reverseScale
import org.firstinspires.ftc.teamcode.lib.hardware.scaleTo
import org.firstinspires.ftc.teamcode.lib.units.Time
import kotlin.math.abs
import kotlin.math.sign

class TargetServo(
    val innerServo: Servo,
    val speed: Double = 0.1
) {
    var unscaledTargetPosition = innerServo.unscaledPosition
        set(value) {
            field = value.coerceIn(0.0, 1.0)
        }

    var targetPosition: Double
        get() = unscaledTargetPosition.reverseScale(innerServo.range).coerceIn(0.0, 1.0)
        set(value) {
            unscaledTargetPosition = value.coerceIn(0.0, 1.0).scaleTo(innerServo.range)
        }

    var unscaledPosition
        get() = innerServo.unscaledPosition
        set(value) {
            unscaledTargetPosition = value
            innerServo.unscaledPosition = value
        }

    var position
        get() = innerServo.position
        set(value) {
            targetPosition = value
            innerServo.position = value
        }

    fun update(deltaTime: Time) {
        val error = unscaledTargetPosition - unscaledPosition
        if (error == 0.0) return
        val step = speed * deltaTime.s
        if (abs(error) < step) {
            unscaledPosition += error
            innerServo.forcePositionUpdate()
        } else {
            unscaledPosition += error.sign * step
        }
    }
}