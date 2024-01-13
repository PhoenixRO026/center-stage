package com.phoenix_ro026.phoenixlib.units

sealed interface Distance {
    val value: Double

    fun <T : Distance> to(unit: T) : T {
        @Suppress("USELESS_CAST", "UNCHECKED_CAST")
        return when(unit as Distance) {
            is Centimeters -> toCentimeters()
            is Inches -> toInches()
            is Meters -> toMeters()
            is Millimeters -> toMillimeters()
        } as T
    }

    operator fun plus(other: Distance) = clone(value + other.to(this).value)
    operator fun minus(other: Distance) = clone(value - other.to(this).value)
    operator fun times(other: Number) = clone(value * other.toDouble())
    operator fun div(other: Number) = clone(value / other.toDouble())
    operator fun div(other: Duration) = Velocity(this, other)
    operator fun unaryMinus() = clone(-value)

    fun toMillimeters(): Millimeters
    fun toCentimeters(): Centimeters
    fun toMeters(): Meters
    fun toInches(): Inches

    override fun toString(): String
    fun unitToString(): String
}

fun <T : Distance> T.clone(newValue: Double) : T {
    @Suppress("USELESS_CAST", "UNCHECKED_CAST")
    return when(this as Distance) {
        is Centimeters -> Centimeters(newValue)
        is Inches -> Inches(newValue)
        is Meters -> Meters(newValue)
        is Millimeters -> Millimeters(newValue)
    } as T
}

@JvmField
val MM = 1.mm
val Number.mm get() = Millimeters(toDouble())
fun mm(number: Double) = number.mm
class Millimeters(override val value: Double) : Distance {
    override fun toMillimeters(): Millimeters = this
    override fun toCentimeters(): Centimeters = Centimeters(value / 10.0)
    override fun toMeters(): Meters = Meters(value / 1000.0)
    override fun toInches(): Inches = Inches(value / 25.4)

    override operator fun plus(other: Distance) : Millimeters = super.plus(other) as Millimeters
    override operator fun minus(other: Distance) : Millimeters = super.minus(other) as Millimeters
    override operator fun times(other: Number) : Millimeters = super.times(other) as Millimeters
    override operator fun div(other: Number) : Millimeters = super.div(other) as Millimeters

    override fun toString(): String = "$value millimeters"
    override fun unitToString(): String = "millimeter"
}

@JvmField
val CM = 1.cm
val Number.cm get() = Centimeters(toDouble())
fun cm(number: Double) = number.cm
class Centimeters(override val value: Double) : Distance {
    override fun toMillimeters(): Millimeters = Millimeters(value * 10.0)
    override fun toCentimeters(): Centimeters = this
    override fun toMeters(): Meters = Meters(value / 100.0)
    override fun toInches(): Inches = Inches(value / 2.54)

    override operator fun plus(other: Distance) : Centimeters = super.plus(other) as Centimeters
    override operator fun minus(other: Distance) : Centimeters = super.minus(other) as Centimeters
    override operator fun times(other: Number) : Centimeters = super.times(other) as Centimeters
    override operator fun div(other: Number) : Centimeters = super.div(other) as Centimeters

    override fun toString(): String = "$value centimeters"
    override fun unitToString(): String = "centimeter"
}

@JvmField
val METER = 1.meters
val Number.meters get() = Meters(toDouble())
fun meters(number: Double) = number.meters
class Meters(override val value: Double) : Distance {
    override fun toMillimeters(): Millimeters = Millimeters(value * 1000.0)
    override fun toCentimeters(): Centimeters = Centimeters(value * 100.0)
    override fun toMeters(): Meters = this
    override fun toInches(): Inches = Inches(value / 0.0254)

    override operator fun plus(other: Distance) : Meters = super.plus(other) as Meters
    override operator fun minus(other: Distance) : Meters = super.minus(other) as Meters
    override operator fun times(other: Number) : Meters = super.times(other) as Meters
    override operator fun div(other: Number) : Meters = super.div(other) as Meters

    override fun toString(): String = "$value meters"
    override fun unitToString(): String = "meter"
}

@JvmField
val INCH = 1.inch
val Number.inch get() = Inches(toDouble())
fun inch(number: Double) = number.inch

class Inches(override val value: Double) : Distance {
    override fun toMillimeters(): Millimeters = Millimeters(value * 25.4)
    override fun toCentimeters(): Centimeters = Centimeters(value * 2.54)
    override fun toMeters(): Meters = Meters(value * 0.0254)
    override fun toInches(): Inches = this

    override operator fun plus(other: Distance) : Inches = super.plus(other) as Inches
    override operator fun minus(other: Distance) : Inches = super.minus(other) as Inches
    override operator fun times(other: Number) : Inches = super.times(other) as Inches
    override operator fun div(other: Number) : Inches = super.div(other) as Inches

    override fun toString(): String = "$value inches"
    override fun unitToString(): String = "inch"
}