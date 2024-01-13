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
}

fun <T : Angle> T.clone(newValue: Double) : T {
    @Suppress("USELESS_CAST", "UNCHECKED_CAST")
    return when(this as Angle) {
        is Degrees -> toDegrees()
        is Radians -> toRadians()
        is Revolutions -> toRevolutions()
    } as T
}

class Degrees(override val value: Double) : Angle {
    override fun toDegrees(): Degrees = this
    override fun toRadians(): Radians = Radians(value / 180.0 * Math.PI)
    override fun toRevolutions(): Revolutions = Revolutions(value / 360.0)
}

class Radians(override val value: Double) : Angle {
    override fun toDegrees(): Degrees = Degrees(value * 180.0 / Math.PI)
    override fun toRadians(): Radians = this
    override fun toRevolutions(): Revolutions = Revolutions(value / 2.0 / Math.PI)
}

class Revolutions(override val value: Double) : Angle {
    override fun toDegrees(): Degrees = Degrees(value * 360.0)
    override fun toRadians(): Radians = Radians(value * 2.0 * PI)
    override fun toRevolutions(): Revolutions = this
}