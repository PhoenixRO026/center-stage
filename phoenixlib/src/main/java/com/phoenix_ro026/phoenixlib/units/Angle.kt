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

    fun toDegrees(): Degrees
    fun toRadians(): Radians
    fun toRevolutions(): Revolutions
    override fun toString(): String
}

fun <T : Angle> T.clone(newValue: Double) : T {
    @Suppress("USELESS_CAST", "UNCHECKED_CAST")
    return when(this as Angle) {
        is Degrees -> toDegrees()
        is Radians -> toRadians()
        is Revolutions -> toRevolutions()
    } as T
}

@JvmField
val DEG = 1.deg
val Number.deg get() = Degrees(toDouble())
fun deg(number: Double) = number.deg
class Degrees(override val value: Double) : Angle {
    override fun toDegrees(): Degrees = this
    override fun toRadians(): Radians = Radians(value / 180.0 * Math.PI)
    override fun toRevolutions(): Revolutions = Revolutions(value / 360.0)

    override operator fun plus(other: Angle) : Degrees = super.plus(other) as Degrees
    override operator fun minus(other: Angle) : Degrees = super.minus(other) as Degrees
    override operator fun times(other: Number) : Degrees = super.times(other) as Degrees
    override operator fun div(other: Number) : Degrees = super.div(other) as Degrees

    override fun toString(): String = "$value degrees"
}

@JvmField
val RAD = 1.rad
val Number.rad get() = Radians(toDouble())
fun rad(number: Double) = number.rad
class Radians(override val value: Double) : Angle {
    override fun toDegrees(): Degrees = Degrees(value * 180.0 / Math.PI)
    override fun toRadians(): Radians = this
    override fun toRevolutions(): Revolutions = Revolutions(value / 2.0 / Math.PI)

    override operator fun plus(other: Angle) : Radians = super.plus(other) as Radians
    override operator fun minus(other: Angle) : Radians = super.minus(other) as Radians
    override operator fun times(other: Number) : Radians = super.times(other) as Radians
    override operator fun div(other: Number) : Radians = super.div(other) as Radians

    override fun toString(): String = "$value radians"
}

@JvmField
val REV = 1.rev
val Number.rev get() = Revolutions(toDouble())
fun rev(number: Double) = number.rev
class Revolutions(override val value: Double) : Angle {
    override fun toDegrees(): Degrees = Degrees(value * 360.0)
    override fun toRadians(): Radians = Radians(value * 2.0 * PI)
    override fun toRevolutions(): Revolutions = this

    override operator fun plus(other: Angle) : Revolutions = super.plus(other) as Revolutions
    override operator fun minus(other: Angle) : Revolutions = super.minus(other) as Revolutions
    override operator fun times(other: Number) : Revolutions = super.times(other) as Revolutions
    override operator fun div(other: Number) : Revolutions = super.div(other) as Revolutions

    override fun toString(): String = "$value revolutions"
}