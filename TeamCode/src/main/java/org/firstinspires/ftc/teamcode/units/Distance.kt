package org.firstinspires.ftc.teamcode.units

sealed interface Distance {
    val value: Double
    fun toCentimeters(): Double
    fun toMilimeters(): Double
    fun toInches(): Double
    fun toMeters(): Double
}

val Number.centimiters: Centimeters
    get() = Centimeters(this.toDouble())

val Number.milimeters: Milimeters
    get() = Milimeters(this.toDouble())

val Number.inches: Inches
    get() = Inches(this.toDouble())

val Number.meters: Meters
    get() = Meters(this.toDouble())

val centimeter = Centimeters(1.0)
val milimeter = Milimeters(1.0)
val inch = Inches(1.0)
val meter = Meters(1.0)

class Centimeters(override val value: Double = 0.0): Distance {

    override fun toCentimeters(): Double {
        return value
    }

    override fun toMilimeters(): Double {
        return value * 10.0
    }

    override fun toInches(): Double {
        return value / 2.54
    }

    override fun toMeters(): Double {
        return value / 100.0
    }
}

class Milimeters(override val value: Double = 0.0): Distance {
    override fun toCentimeters(): Double {
        return value / 10.0
    }

    override fun toMilimeters(): Double {
        return value
    }

    override fun toInches(): Double {
        return value / 25.4
    }

    override fun toMeters(): Double {
        return value / 1000.0
    }
}

class Inches(override val value: Double = 0.0): Distance {

    override fun toCentimeters(): Double {
        return value * 2.54
    }

    override fun toMilimeters(): Double {
        return value * 25.4
    }

    override fun toInches(): Double {
        return value
    }

    override fun toMeters(): Double {
        return value * 0.0254
    }
}

class Meters(override val value: Double = 0.0): Distance {

    override fun toCentimeters(): Double {
        return value * 100.0
    }

    override fun toMilimeters(): Double {
        return value * 1000.0
    }

    override fun toInches(): Double {
        return value / 0.0254
    }

    override fun toMeters(): Double {
        return value
    }
}
