@file:Suppress("unused")

package com.phoenix.phoenixlib.units

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d

data class Distance(
        @JvmField var inch: Double
) {
    val cm get() = inch * 2.54
    val mm get() = cm * 10.0
    val m get() = cm / 100.0

    operator fun plus(other: Distance) = Distance(inch + other.inch)
    operator fun minus(other: Distance) = Distance(inch - other.inch)
    operator fun times(scalar: Number) = Distance(inch * scalar.toDouble())
    operator fun div(scalar: Number) = Distance(inch / scalar.toDouble())
    operator fun div(other: Distance) = inch / other.inch
    operator fun div(time: Time) = Velocity(this, time)
    operator fun unaryMinus() = Distance(-inch)

    override fun toString() = "$inch " + if (inch == 1.0) "inch" else "inches"
}

val Number.m get() = Distance(toDouble() / 0.0254)
val Number.mm get() = Distance(toDouble() / 25.4)
val Number.cm get() = Distance(toDouble() / 2.54)
val Number.inch get() = Distance(toDouble())

fun m(number: Double) = number.m
fun mm(number: Double) = number.mm
fun cm(number: Double) = number.cm
fun inch(number: Double) = number.inch

@JvmField val m = 1.m
@JvmField val mm = 1.mm
@JvmField val cm = 1.cm
@JvmField val inch = 1.inch

@Suppress("FunctionName")
fun Vector2d(x: Distance, y: Distance) = Distance2d(x, y)

data class Distance2d(@JvmField var x: Distance, @JvmField var y: Distance) {
    operator fun plus(other: Distance2d) = Distance2d(x + other.x, y + other.y)
    operator fun minus(other: Distance2d) = Distance2d(x - other.x, y - other.y)
    operator fun times(scalar: Number) = Distance2d(x * scalar, y * scalar)
    operator fun div(scalar: Number) = Distance2d(x / scalar, y / scalar)
    operator fun unaryMinus() = Distance2d(-x, -y)

    val meters get() = Vector2d(x.m, y.m)
    val mm get() = Vector2d(x.mm, y.mm)
    val cm get() = Vector2d(x.cm, y.cm)
    val inch get() = Vector2d(x.inch, y.inch)

    val vector2d get() = inch
}

val Vector2d.m get() = Distance2d(x.m, y.m)
val Vector2d.mm get() = Distance2d(x.mm, y.mm)
val Vector2d.cm get() = Distance2d(x.cm, y.cm)
val Vector2d.inch get() = Distance2d(x.inch, y.inch)
val Vector2d.dist get() = inch

fun m(vector: Vector2d) = vector.m
fun mm(vector: Vector2d) = vector.mm
fun cm(vector: Vector2d) = vector.cm
fun inch(vector: Vector2d) = vector.inch
fun dist(vector: Vector2d) = vector.dist

@Suppress("FunctionName")
fun Pose2d(position: Distance2d, heading: Angle) = Pose(position, heading)

@Suppress("FunctionName")
fun Pose2d(x: Distance, y: Distance, heading: Angle) = Pose(x, y, heading)

data class Pose(@JvmField var position: Distance2d, @JvmField var heading: Angle) {
    constructor(x: Distance, y: Distance, heading: Angle) : this(Distance2d(x, y), heading)

    operator fun plus(other: Pose) = Pose(position + other.position, heading + other.heading)
    operator fun plus(other: Distance2d) = Pose(position + other, heading)
    operator fun plus(other: Angle) = Pose(position, heading + other)
    operator fun minus(other: Pose) = Pose(position - other.position, heading - other.heading)
    operator fun minus(other: Distance2d) = Pose(position - other, heading)
    operator fun minus(other: Angle) = Pose(position, heading - other)

    val pose2d get() = Pose2d(position.inch, heading.rad)
}