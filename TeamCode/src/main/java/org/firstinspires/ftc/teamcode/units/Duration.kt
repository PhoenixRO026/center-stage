@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.units

sealed class Duration(val value: Double) {
    abstract fun toSeconds(): Double
    abstract fun toMinutes(): Double
    abstract override fun toString(): String

    fun clone(newValue: Double) = when (this) {
        is Minutes -> Minutes(newValue)
        is Seconds -> Seconds(newValue)
    }
    fun toUnit(d: Duration) = when(d) {
        is Minutes -> toMinutes()
        is Seconds -> toSeconds()
    }
    fun asUnit(unit: Duration) = unit.clone(toUnit(unit))
    operator fun plus(d: Duration) = clone(value + d.toUnit(this))
    operator fun minus(d: Duration) = clone(value - d.toUnit(this))
    operator fun unaryMinus() = clone(-value)
    operator fun times(z: Double) = clone(value * z)
    operator fun div(z: Double) = clone(value / z)
}

val Number.seconds get() = Seconds(this.toDouble())
val Number.minutes get() = Minutes(this.toDouble())

val second = Seconds(1.0)
val minute = Minutes(1.0)

class Seconds(value: Double = 0.0): Duration(value) {
    override fun toSeconds(): Double = value
    override fun toMinutes(): Double = value / 60.0
    override fun toString(): String = "$value seconds"
}

class Minutes(value: Double = 0.0): Duration(value) {
    override fun toSeconds(): Double = value * 60.0
    override fun toMinutes(): Double = value
    override fun toString(): String = "$value minutes"
}
