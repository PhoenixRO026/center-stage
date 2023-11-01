package org.firstinspires.ftc.teamcode.units

sealed interface Duration {
    val value: Double

    fun toSeconds(): Double
    fun toMinutes(): Double
}

val Number.seconds: Seconds
    get() = Seconds(this.toDouble())

val Number.minutes: Minutes
    get() = Minutes(this.toDouble())

val second = Seconds(1.0)
val minute = Minutes(1.0)

class Seconds(override val value: Double = 0.0): Duration {
    override fun toSeconds(): Double {
        return value
    }

    override fun toMinutes(): Double {
        return value / 60.0
    }
}

class Minutes(override val value: Double = 0.0): Duration {
    override fun toSeconds(): Double {
        return value * 60.0
    }

    override fun toMinutes(): Double {
        return value
    }

}
