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

    operator fun plus(other: Angle) = clone(value + other.to(this).value)
    operator fun minus(other: Angle) = clone(value - other.to(this).value)
    operator fun times(other: Number) = clone(value * other.toDouble())
    operator fun div(other: Number) = clone(value / other.toDouble())
    operator fun unaryMinus() = clone(-value)
    operator fun <U: Duration> div(other: U) = AngularVelocity(this, other)

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

fun <T: Angle> T.plusT(other: Angle) = clone(value + other.to(this).value)
fun <T: Angle> T.minusT(other: Angle) = clone(value - other.to(this).value)
fun <T: Angle> T.timesT(other: Number) = clone(value * other.toDouble())
fun <T: Angle> T.divT(other: Number) = clone(value / other.toDouble())
fun <T: Angle> T.unaryMinusT() = clone(-value)
fun <T: Angle, U: Duration> T.divT(other: U) = AngularVelocity(this, other)

@JvmField
val DEG = 1.deg
val Number.deg get() = Degrees(toDouble())
fun deg(number: Double) = number.deg
class Degrees(override val value: Double) : Angle {
    override fun toDegrees(): Degrees = this
    override fun toRadians(): Radians = Radians(value / 180.0 * Math.PI)
    override fun toRevolutions(): Revolutions = Revolutions(value / 360.0)

    override operator fun plus(other: Angle) = clone(value + other.to(this).value)
    override operator fun minus(other: Angle) = clone(value - other.to(this).value)
    override operator fun times(other: Number) = clone(value * other.toDouble())
    override operator fun div(other: Number) = clone(value / other.toDouble())
    override operator fun unaryMinus() = clone(-value)
    override operator fun <U: Duration> div(other: U) = AngularVelocity(this, other)

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

    override operator fun plus(other: Angle) = clone(value + other.to(this).value)
    override operator fun minus(other: Angle) = clone(value - other.to(this).value)
    override operator fun times(other: Number) = clone(value * other.toDouble())
    override operator fun div(other: Number) = clone(value / other.toDouble())
    override operator fun unaryMinus() = clone(-value)
    override operator fun <U: Duration> div(other: U) = AngularVelocity(this, other)

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

    override operator fun plus(other: Angle) = clone(value + other.to(this).value)
    override operator fun minus(other: Angle) = clone(value - other.to(this).value)
    override operator fun times(other: Number) = clone(value * other.toDouble())
    override operator fun div(other: Number) = clone(value / other.toDouble())
    override operator fun unaryMinus() = clone(-value)
    override operator fun <U: Duration> div(other: U) = AngularVelocity(this, other)

    override fun toString(): String = "$value ${unitToString()}" + if (value != 1.0) "s" else ""
    override fun unitToString(): String = "revolution"
}