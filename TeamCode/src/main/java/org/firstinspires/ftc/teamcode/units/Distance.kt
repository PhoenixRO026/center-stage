@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.units

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d

sealed class Distance(val value: Double) {
    abstract fun toCentimeters(): Double
    abstract fun toMilimeters(): Double
    abstract fun toInches(): Double
    abstract fun toMeters(): Double
    abstract override fun toString(): String

    fun clone(newValue: Double) = when(this) {
        is Centimeters -> Centimeters(newValue)
        is Inches -> Inches(newValue)
        is Meters -> Meters(newValue)
        is Milimeters -> Milimeters(newValue)
    }
    fun toUnit(d: Distance) = when(d) {
        is Centimeters -> toCentimeters()
        is Inches -> toInches()
        is Meters -> toMeters()
        is Milimeters -> toMilimeters()
    }
    fun asUnit(unit: Distance) = unit.clone(toUnit(unit))
    operator fun plus(d: Distance) = clone(value + d.toUnit(this))
    operator fun minus(d: Distance) = clone(value - d.toUnit(this))
    operator fun unaryMinus() = clone(-value)
    operator fun times(z: Double) = clone(value * z)
    operator fun div(z: Double) = clone(value / z)
}

val Number.cm get() = Centimeters(this.toDouble())
val Number.mm get() = Milimeters(this.toDouble())
val Number.inch get() = Inches(this.toDouble())
val Number.meters get() = Meters(this.toDouble())

val centimeter = Centimeters(1.0)
val milimeter = Milimeters(1.0)
val inch = Inches(1.0)
val meter = Meters(1.0)

class Centimeters(value: Double = 0.0): Distance(value) {
    override fun toCentimeters(): Double = value
    override fun toMilimeters(): Double = value * 10.0
    override fun toInches(): Double = value / 2.54
    override fun toMeters(): Double = value / 100.0
    override fun toString() = "$value centimeters"
}

class Milimeters(value: Double= 0.0): Distance(value) {
    override fun toCentimeters(): Double = value / 10.0
    override fun toMilimeters(): Double = value
    override fun toInches(): Double = value / 25.4
    override fun toMeters(): Double = value / 1000.0
    override fun toString() = "$value milimeters"
}

class Inches(value: Double = 0.0): Distance(value) {
    override fun toCentimeters(): Double = value * 2.54
    override fun toMilimeters(): Double = value * 25.4
    override fun toInches(): Double = value
    override fun toMeters(): Double = value * 0.0254
    override fun toString() = "$value inches"
}

class Meters(value: Double): Distance(value) {
    override fun toCentimeters(): Double = value * 100.0
    override fun toMilimeters(): Double = value * 1000.0
    override fun toInches(): Double = value / 0.0254
    override fun toMeters(): Double = value
    override fun toString() = "$value meters"
}

fun vector2d(x: Distance, y: Distance) = Distance2d(x, y)

data class Distance2d(@JvmField val x: Distance, @JvmField val y: Distance) {
    fun toVector2d() = Vector2d(x.toInches(), y.toInches())
    operator fun plus(d: Distance2d) = Distance2d(x + d.x, y + d.y)
    operator fun minus(d: Distance2d) = Distance2d(x - d.x, y - d.y)
    operator fun unaryMinus() = Distance2d(-x, -y)
    operator fun times(z: Double) = Distance2d(x * z, y * z)
    operator fun div(z: Double) = Distance2d(x / z, y / z)
}

fun pose2d(position: Distance2d, heading: Rotation) = Pose(position, heading)
fun pose2d(x: Distance, y: Distance, heading: Rotation) = Pose(x, y, heading)

data class Pose(@JvmField val position: Distance2d, @JvmField val heading: Rotation) {
    constructor(x: Distance, y: Distance, heading: Rotation) : this(Distance2d(x, y), heading)
    fun toPose2d() = Pose2d(position.toVector2d(), heading.toRotation2d())
    operator fun plus(p: Pose) = Pose(position + p.position, heading + p.heading)
    operator fun minus(p: Pose) = Pose(position - p.position, heading - p.heading)
    operator fun plus(d: Distance2d) = Pose(position + d, heading)
    operator fun minus(d: Distance2d) = Pose(position - d, heading)
    operator fun plus(r: Rotation) = Pose(position, heading + r)
    operator fun minus(r: Rotation) = Pose(position, heading - r)
    operator fun unaryMinus() = Pose(-position, -heading)
    operator fun times(z: Double) = Pose(position * z, heading * z)
    operator fun div(z: Double) = Pose(position / z, heading / z)
}