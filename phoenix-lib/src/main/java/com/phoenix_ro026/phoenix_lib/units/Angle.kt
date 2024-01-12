package com.phoenix_ro026.phoenix_lib.units

import kotlin.math.PI

sealed interface Angle {
    val value: Double
    @Suppress("UNCHECKED_CAST")
    fun <T: Angle> to(unit: T): T {
        @Suppress("USELESS_CAST")
        return when(unit as Angle) {
            is Degree -> toDegrees() as T
            is Radian -> toRadians() as T
            is Revolution -> toRevolutions() as T
        }
    }
    operator fun plus(other: Angle) = clone(value + other.to(this).value)
    operator fun minus(other: Angle) = clone(value - other.to(this).value)
    operator fun times(other: Number) = clone(value * other.toDouble())
    operator fun div(other: Number) = clone(value / other.toDouble())
    fun toRadians(): Radian
    fun toDegrees(): Degree
    fun toRevolutions(): Revolution
}

@Suppress("UNCHECKED_CAST")
private fun <T: Angle> T.clone(newValue: Double): T {
    @Suppress("USELESS_CAST")
    return when(this as Angle) {
        is Degree -> Degree(newValue) as T
        is Radian -> Radian(newValue) as T
        is Revolution -> Revolution(newValue) as T
    }
}

@JvmField
val RAD = 1.rad
val Number.rad get() = Radian(toDouble())
fun rad(number: Double) = number.rad
class Radian(override val value: Double) : Angle {
    override fun toRadians(): Radian = this
    override fun toDegrees(): Degree = Degree(value * 180.0 / Math.PI)
    override fun toRevolutions(): Revolution = Revolution(value / (2.0 * PI))
    override operator fun plus(other: Angle): Radian {
        return super.plus(other) as Radian
    }
    override operator fun minus(other: Angle): Radian {
        return super.minus(other) as Radian
    }
    override operator fun times(other: Number): Radian {
        return super.times(other) as Radian
    }
    override operator fun div(other: Number): Radian {
        return super.div(other) as Radian
    }
}

@JvmField
val DEG = 1.deg
val Number.deg get() = Degree(toDouble())
fun deg(number: Double) = number.deg
class Degree(override val value: Double) : Angle {
    override fun toRadians(): Radian = Radian(value / 180.0 * Math.PI)
    override fun toDegrees(): Degree = this
    override fun toRevolutions(): Revolution = Revolution(value / 360.0)
    override operator fun plus(other: Angle): Degree {
        return super.plus(other) as Degree
    }
    override operator fun minus(other: Angle): Degree {
        return super.minus(other) as Degree
    }
    override operator fun times(other: Number): Degree {
        return super.times(other) as Degree
    }
    override operator fun div(other: Number): Degree {
        return super.div(other) as Degree
    }
}

@JvmField
val REV = 1.rev
val Number.rev get() = Revolution(toDouble())
fun rev(number: Double) = number.rev
class Revolution(override val value: Double) : Angle {
    override fun toRadians(): Radian = Radian(value * 2.0 * PI)
    override fun toDegrees(): Degree = Degree(value * 360.0)
    override fun toRevolutions(): Revolution = this
    override operator fun plus(other: Angle): Revolution {
        return super.plus(other) as Revolution
    }
    override operator fun minus(other: Angle): Revolution {
        return super.minus(other) as Revolution
    }
    override operator fun times(other: Number): Revolution {
        return super.times(other) as Revolution
    }
    override operator fun div(other: Number): Revolution {
        return super.div(other) as Revolution
    }
}