@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.units

import com.acmerobotics.roadrunner.Rotation2d
import kotlin.math.PI

sealed class Rotation(val value: Double) {
    abstract fun toDegrees(): Double
    abstract fun toRadians(): Double
    abstract fun toRevolutions(): Double
    abstract override fun toString(): String

    fun clone(newValue: Double) = when(this) {
        is Degrees -> Degrees(newValue)
        is Radians -> Radians(newValue)
        is Revolutions -> Revolutions(newValue)
    }
    fun toUnit(r: Rotation) = when(r) {
        is Degrees -> toDegrees()
        is Radians -> toRadians()
        is Revolutions -> toRevolutions()
    }
    fun toRotation2d() = Rotation2d.exp(toUnit(radian))
    fun asUnit(unit: Rotation) = unit.clone(toUnit(unit))
    operator fun plus(r: Rotation) = clone(value + r.toUnit(this))
    operator fun minus(r: Rotation) = clone(value - r.toUnit(this))
    operator fun unaryMinus() = clone(-value)
    operator fun times(z: Double) = clone(value * z)
    operator fun div(z: Double) = clone(value / z)
}

val Number.degrees get() = Degrees(this.toDouble())
val Number.radians get() = Radians(this.toDouble())
val Number.revolutions get() = Revolutions(this.toDouble())

val degree = Degrees(1.0)
val radian = Radians(1.0)
val revolution = Revolutions(1.0)

class Degrees(value: Double = 0.0): Rotation(value) {
    override fun toDegrees(): Double = value
    override fun toRadians(): Double = value * (PI / 180.0)
    override fun toRevolutions(): Double = value / 360.0
    override fun toString(): String = "$value degrees"
}

class Radians(value: Double = 0.0): Rotation(value) {
    override fun toDegrees(): Double = value * (180.0 / PI)
    override fun toRadians(): Double = value
    override fun toRevolutions(): Double = value / (2.0 * PI)
    override fun toString(): String = "$value radians"
}

class Revolutions(value: Double = 0.0): Rotation(value) {
    override fun toDegrees(): Double = value * 360.0
    override fun toRadians(): Double = value * (2.0 * PI)
    override fun toRevolutions(): Double = value
    override fun toString(): String = "$value revolutions"
}
