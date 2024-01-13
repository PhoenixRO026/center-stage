package com.phoenix_ro026.phoenixlib.units

class AngularVelocity(val angle: Angle, val duration: Duration) {
    fun to(unit: AngularVelocity): AngularVelocity {
        val angleVal = angle.to(unit.angle)
        val durationVal = duration.to(unit.duration)
        return angleVal / durationVal
    }
    fun distanceAsUnit(unit: AngularVelocity): Angle {
        //2 / 1 = 2    1 meter per 2 seconds = 0.5 meter per 1 second
        val durationRatio = duration.to(unit.duration).value / unit.duration.value
        return angle.to(unit.angle) / durationRatio
    }

    fun durationAsUnit(unit: AngularVelocity): Duration {
        //2 / 1 = 2    1 meter per 2 seconds = 0.5 meter per 1 second
        val angleRatio = angle.to(unit.angle).value / unit.angle.value
        return duration.to(unit.duration) / angleRatio
    }

    operator fun plus(other: AngularVelocity) = AngularVelocity(angle + other.distanceAsUnit(this), duration)
    operator fun minus(other: AngularVelocity) = AngularVelocity(angle - other.distanceAsUnit(this), duration)
    operator fun unaryMinus() = AngularVelocity(-angle, duration)
    operator fun times(other: Number) = AngularVelocity(angle * other, duration)
    operator fun div(other: Number) = AngularVelocity(angle / other , duration)
    operator fun times(other: Duration) = distanceAsUnit(AngularVelocity(angle, other))
    operator fun div(other: Angle) = durationAsUnit(AngularVelocity(other, duration))

    override fun toString(): String = "$angle per " + if (duration.value == 1.0) duration.unitToString() else "$duration"
}