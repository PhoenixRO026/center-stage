package org.firstinspires.ftc.teamcode.units

import kotlin.math.PI

sealed interface Rotation {
    val value: Double

    fun toDegrees(): Double
    fun toRadians(): Double
    fun toRevolutions(): Double
}

val Number.degrees: Degrees
    get() = Degrees(this.toDouble())

val Number.radians: Radians
    get() = Radians(this.toDouble())

val Number.revolutions: Revolutions
    get() = Revolutions(this.toDouble())

val degree = Degrees(1.0)
val radian = Radians(1.0)
val revolution = Revolutions(1.0)

class Degrees(override val value: Double = 0.0): Rotation {
    override fun toDegrees(): Double {
        return value
    }

    override fun toRadians(): Double {
        return value * (PI / 180.0)
    }

    override fun toRevolutions(): Double {
        return value / 360.0
    }

}

class Radians(override val value: Double = 0.0): Rotation {
    override fun toDegrees(): Double {
        return value * (180.0 / PI)
    }

    override fun toRadians(): Double {
        return value
    }

    override fun toRevolutions(): Double {
        return value / (2.0 * PI)
    }
}

class Revolutions(override val value: Double = 0.0): Rotation {
    override fun toDegrees(): Double {
        return value * 360.0
    }

    override fun toRadians(): Double {
        return value * (2.0 * PI)
    }

    override fun toRevolutions(): Double {
        return value
    }
}
