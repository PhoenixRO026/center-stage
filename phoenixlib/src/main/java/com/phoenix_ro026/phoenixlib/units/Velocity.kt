package com.phoenix_ro026.phoenixlib.units

class Velocity<out T: Distance, U: Duration>(val distance: T, val duration: U) {
    fun <V: Distance, W: Duration> to(unit: Velocity<V, W>): Velocity<V, W> {
        val distanceVal = distanceAsUnit(unit)
        val durationVal = unit.duration
        return distanceVal / durationVal
    }
    fun <V: Distance, W: Duration> distanceAsUnit(unit: Velocity<V, W>): V {
        //2 / 1 = 2    1 meter per 2 seconds = 0.5 meter per 1 second
        val durationRatio = duration.to(unit.duration).value / unit.duration.value
        return distance.to(unit.distance) / durationRatio
    }

    fun <V: Distance, W: Duration> durationAsUnit(unit: Velocity<V, W>): W {
        //2 / 1 = 2    1 meter per 2 seconds = 0.5 meter per 1 second
        val distanceRatio = distance.to(unit.distance).value / unit.distance.value
        return duration.to(unit.duration) / distanceRatio
    }

    operator fun <V: Distance, W: Duration> plus(other: Velocity<V, W>) = Velocity(distance + other.distanceAsUnit(this), duration)
    operator fun <V: Distance, W: Duration> minus(other: Velocity<V, W>) = Velocity(distance - other.distanceAsUnit(this), duration)
    operator fun unaryMinus() = Velocity(-distance, duration)
    operator fun times(other: Number) = Velocity(distance * other, duration)
    operator fun div(other: Number) = Velocity(distance / other , duration)
    operator fun <V: Duration> times(other: V) = distanceAsUnit(Velocity(distance, other))
    operator fun <V: Distance> div(other: V) = durationAsUnit(Velocity(other, duration))

    override fun toString(): String = "$distance per " + if (duration.value == 1.0) duration.unitToString() else "$duration"
}

@JvmField
val CM_SEC = 1.cmSec
val Number.cmSec get() = cm / SEC
fun cmSec(number: Double) = number.cmSec
@JvmField
val INCH_SEC = 1.inchSec
val Number.inchSec get() = inch / SEC
fun inchSec(number: Double) = number.inchSec
@JvmField
val METER_SEC = 1.meterSec
val Number.meterSec get() = meters / SEC
fun meterSec(number: Double) = number.meterSec
