package com.phoenix_ro026.phoenixlib.units

import kotlin.math.PI

sealed interface Angle {
    val value: Double

    fun <T : Angle> to(unit: T) : T {
        @Suppress("USELESS_CAST", "UNCHECKED_CAST")
        return when(unit as Angle) {
            is Degrees -> toDegrees()
            is Radians -> toRadians()
            is Revolutions -> toRevolutions()
        } as T
    }

    fun plus(other: Angle) = clone(value + other.to(this).value)
    fun minus(other: Angle) = clone(value - other.to(this).value)
    fun times(other: Number) = clone(value * other.toDouble())
    fun div(other: Number) = clone(value / other.toDouble())
    fun unaryMinus() = clone(-value)
    fun <U: Duration> div(other: U) = AngularVelocity(this, other)

    fun toDegrees(): Degrees
    fun toRadians(): Radians
    fun toRevolutions(): Revolutions
    fun unitToString(): String
    override fun toString(): String
}

fun <T : Angle> T.clone(newValue: Double) : T {
    @Suppress("USELESS_CAST", "UNCHECKED_CAST")
    return when(this as Angle) {
        is Degrees -> Degrees(newValue)
        is Radians -> Radians(newValue)
        is Revolutions -> Revolutions(newValue)
    } as T
}

operator fun <T: Angle> T.plus(other: Angle) = clone(value + other.to(this).value)
operator fun <T: Angle> T.minus(other: Angle) = clone(value - other.to(this).value)
operator fun <T: Angle> T.times(other: Number) = clone(value * other.toDouble())
operator fun <T: Angle> T.div(other: Number) = clone(value / other.toDouble())
operator fun <T: Angle> T.unaryMinus() = clone(-value)
operator fun <T: Angle, U: Duration> T.div(other: U) = AngularVelocity(this, other)

@JvmField
val DEG = 1.deg
val Number.deg get() = Degrees(toDouble())
fun deg(number: Double) = number.deg
class Degrees(override val value: Double) : Angle {
    override fun toDegrees(): Degrees = this
    override fun toRadians(): Radians = Radians(value / 180.0 * Math.PI)
    override fun toRevolutions(): Revolutions = Revolutions(value / 360.0)

    override fun plus(other: Angle) = clone(value + other.to(this).value)
    override fun minus(other: Angle) = clone(value - other.to(this).value)
    override fun times(other: Number) = clone(value * other.toDouble())
    override fun div(other: Number) = clone(value / other.toDouble())
    override fun unaryMinus() = clone(-value)
    override fun <U: Duration> div(other: U) = AngularVelocity(this, other)

    override fun toString(): String = "$value ${unitToString()}" + if (value != 1.0) "s" else ""
    override fun unitToString(): String = "degree"
}

@JvmField
val RAD = 1.rad
val Number.rad get() = Radians(toDouble())
fun rad(number: Double) = number.rad
class Radians(override val value: Double) : Angle {
    override fun toDegrees(): Degrees = Degrees(value * 180.0 / Math.PI)
    override fun toRadians(): Radians = this
    override fun toRevolutions(): Revolutions = Revolutions(value / 2.0 / Math.PI)

    override fun plus(other: Angle) = clone(value + other.to(this).value)
    override fun minus(other: Angle) = clone(value - other.to(this).value)
    override fun times(other: Number) = clone(value * other.toDouble())
    override fun div(other: Number) = clone(value / other.toDouble())
    override fun unaryMinus() = clone(-value)
    override fun <U: Duration> div(other: U) = AngularVelocity(this, other)

    override fun toString(): String = "$value ${unitToString()}" + if (value != 1.0) "s" else ""
    override fun unitToString(): String = "radian"
}

@JvmField
val REV = 1.rev
val Number.rev get() = Revolutions(toDouble())
fun rev(number: Double) = number.rev
class Revolutions(override val value: Double) : Angle {
    override fun toDegrees(): Degrees = Degrees(value * 360.0)
    override fun toRadians(): Radians = Radians(value * 2.0 * PI)
    override fun toRevolutions(): Revolutions = this

    override fun plus(other: Angle) = clone(value + other.to(this).value)
    override fun minus(other: Angle) = clone(value - other.to(this).value)
    override fun times(other: Number) = clone(value * other.toDouble())
    override fun div(other: Number) = clone(value / other.toDouble())
    override fun unaryMinus() = clone(-value)
    override fun <U: Duration> div(other: U) = AngularVelocity(this, other)

    override fun toString(): String = "$value ${unitToString()}" + if (value != 1.0) "s" else ""
    override fun unitToString(): String = "revolution"
}