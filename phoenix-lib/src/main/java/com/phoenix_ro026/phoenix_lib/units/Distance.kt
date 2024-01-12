package com.phoenix_ro026.phoenix_lib.units

sealed interface Distance {
    val value: Double
    @Suppress("UNCHECKED_CAST")
    fun <T: Distance> to(unit: T): T {
        @Suppress("USELESS_CAST")
        return when(unit as Distance) {
            is Centimeter -> toCentimeters() as T
            is Milimeter -> toMilimeters() as T
            is Meter -> toMeters() as T
            is Inch -> toInches() as T
        }
    }
    operator fun plus(other: Distance) = clone(value + other.to(this).value)
    operator fun minus(other: Distance) = clone(value - other.to(this).value)
    operator fun times(other: Number) = clone(value * other.toDouble())
    operator fun div(other: Number) = clone(value / other.toDouble())
    override fun toString(): String
    fun toMilimeters(): Milimeter
    fun toCentimeters(): Centimeter
    fun toMeters(): Meter
    fun toInches(): Inch
}

@Suppress("UNCHECKED_CAST")
private fun <T: Distance> T.clone(newValue: Double): T {
    @Suppress("USELESS_CAST")
    return when(this as Distance) {
        is Centimeter -> Centimeter(newValue) as T
        is Inch -> Inch(newValue) as T
        is Meter -> Meter(newValue) as T
        is Milimeter -> Milimeter(newValue) as T
    }
}

@JvmField
val MM = 1.mm
val Number.mm get() = Milimeter(toDouble())
fun mm(number: Double) = number.mm
class Milimeter(override val value: Double) : Distance {
    override fun toMilimeters(): Milimeter = this
    override fun toCentimeters(): Centimeter = Centimeter(value / 10.0)
    override fun toMeters(): Meter = Meter(value / 1000.0)
    override fun toInches(): Inch = Inch(value / 25.4)
    override operator fun plus(other: Distance): Milimeter {
        return super.plus(other) as Milimeter
    }
    override operator fun minus(other: Distance): Milimeter {
        return super.minus(other) as Milimeter
    }
    override operator fun times(other: Number): Milimeter {
        return super.times(other) as Milimeter
    }
    override operator fun div(other: Number): Milimeter {
        return super.div(other) as Milimeter
    }
    override fun toString(): String = "$value milimeters"
}

@JvmField
val CM = 1.cm
val Number.cm get() = Centimeter(toDouble())
fun cm(number: Double) = number.cm
class Centimeter(override val value: Double) : Distance {
    override fun toMilimeters(): Milimeter = Milimeter(value * 10.0)
    override fun toCentimeters(): Centimeter = this
    override fun toMeters(): Meter = Meter(value / 100.0)
    override fun toInches(): Inch = Inch(value / 2.54)
    override operator fun plus(other: Distance): Centimeter {
        return super.plus(other) as Centimeter
    }
    override operator fun minus(other: Distance): Centimeter {
        return super.minus(other) as Centimeter
    }
    override operator fun times(other: Number): Centimeter {
        return super.times(other) as Centimeter
    }
    override operator fun div(other: Number): Centimeter {
        return super.div(other) as Centimeter
    }
    override fun toString(): String = "$value centimeters"
}

@JvmField
val METER = 1.meters
val Number.meters get() = Meter(toDouble())
fun meters(number: Double) = number.meters
class  Meter(override val value: Double) : Distance {
    override fun toMilimeters(): Milimeter = Milimeter(value * 1000.0)
    override fun toCentimeters(): Centimeter = Centimeter(value * 100.0)
    override fun toMeters(): Meter = this
    override fun toInches(): Inch = Inch(value / 0.0254)
    override operator fun plus(other: Distance): Meter {
        return super.plus(other) as Meter
    }
    override operator fun minus(other: Distance): Meter {
        return super.minus(other) as Meter
    }
    override operator fun times(other: Number): Meter {
        return super.times(other) as Meter
    }
    override operator fun div(other: Number): Meter {
        return super.div(other) as Meter
    }
    override fun toString(): String = "$value meters"
}

@JvmField
val INCH = 1.inch
val Number.inch get() = Inch(toDouble())
fun inches(number: Double) = number.inch
class Inch(override val value: Double) : Distance {
    override fun toMilimeters(): Milimeter = Milimeter(value * 25.4)
    override fun toCentimeters(): Centimeter = Centimeter(value * 2.54)
    override fun toMeters(): Meter = Meter(value * 0.0254)
    override fun toInches(): Inch = this
    override operator fun plus(other: Distance): Inch {
        return super.plus(other) as Inch
    }
    override operator fun minus(other: Distance): Inch {
        return super.minus(other) as Inch
    }
    override operator fun times(other: Number): Inch {
        return super.times(other) as Inch
    }
    override operator fun div(other: Number): Inch {
        return super.div(other) as Inch
    }
    override fun toString(): String = "$value inches"
}

