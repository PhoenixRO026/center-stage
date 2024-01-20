package com.phoenix_ro026.phoenixlib.units

sealed interface Duration {
    val value: Double

    fun <T : Duration> to(unit: T) : T {
        @Suppress("USELESS_CAST", "UNCHECKED_CAST")
        return when(unit as Duration) {
            is Milliseconds -> toMilliseconds()
            is Minutes -> toMinutes()
            is Seconds -> toSeconds()
        } as T
    }

    operator fun plus(other: Duration) = clone(value + other.to(this).value)
    operator fun minus(other: Duration) = clone(value - other.to(this).value)
    operator fun times(other: Number) = clone(value * other.toDouble())
    operator fun div(other: Number) = clone(value / other.toDouble())
    operator fun unaryMinus() = clone(-value)

    fun toSeconds(): Seconds
    fun toMilliseconds(): Milliseconds
    fun toMinutes(): Minutes

    override fun toString(): String
    fun unitToString(): String
}

fun <T : Duration> T.clone(newValue: Double) : T {
    @Suppress("USELESS_CAST", "UNCHECKED_CAST")
    return when(this as Duration) {
        is Milliseconds -> Milliseconds(newValue)
        is Minutes -> Minutes(newValue)
        is Seconds -> Seconds(newValue)
    } as T
}

fun <T: Duration> T.plusT(other: Duration) = clone(value + other.to(this).value)
fun <T: Duration> T.minusT(other: Duration) = clone(value - other.to(this).value)
fun <T: Duration> T.timesT(other: Number) = clone(value * other.toDouble())
fun <T: Duration> T.divT(other: Number) = clone(value / other.toDouble())
fun <T: Duration> T.unaryMinusT() = clone(-value)

@JvmField
val SEC= 1.sec
val Number.sec get() = Seconds(toDouble())
fun sec(number: Double) = number.sec
class Seconds(override val value: Double) : Duration {
    override fun toSeconds(): Seconds = this
    override fun toMilliseconds(): Milliseconds = Milliseconds(value * 1000.0)
    override fun toMinutes(): Minutes = Minutes(value / 60.0)

    override operator fun plus(other: Duration) = clone(value + other.to(this).value)
    override operator fun minus(other: Duration) = clone(value - other.to(this).value)
    override operator fun times(other: Number) = clone(value * other.toDouble())
    override operator fun div(other: Number) = clone(value / other.toDouble())
    override operator fun unaryMinus() = clone(-value)

    override fun toString(): String = "$value ${unitToString()}" + if (value != 1.0) "s" else ""
    override fun unitToString(): String = "second"
}

@JvmField
val MILLISEC = 1.millisec
val Number.millisec get() = Milliseconds(toDouble())
fun millisec(number: Double) = number.millisec
class Milliseconds(override val value: Double) : Duration {
    override fun toSeconds(): Seconds = Seconds(value / 1000.0)
    override fun toMilliseconds(): Milliseconds = this
    override fun toMinutes(): Minutes = Minutes(value / 1000.0 / 60.0)

    override operator fun plus(other: Duration) = clone(value + other.to(this).value)
    override operator fun minus(other: Duration) = clone(value - other.to(this).value)
    override operator fun times(other: Number) = clone(value * other.toDouble())
    override operator fun div(other: Number) = clone(value / other.toDouble())
    override operator fun unaryMinus() = clone(-value)

    override fun toString(): String = "$value ${unitToString()}" + if (value != 1.0) "s" else ""
    override fun unitToString(): String = "millisecond"
}

@JvmField
val MIN = 1.min
val Number.min get() = Minutes(toDouble())
fun min(number: Double) = number.min
class Minutes(override val value: Double) : Duration {
    override fun toSeconds(): Seconds = Seconds(value * 60.0)
    override fun toMilliseconds(): Milliseconds = Milliseconds(value * 1000.0 * 60.0)
    override fun toMinutes(): Minutes = this

    override operator fun plus(other: Duration) = clone(value + other.to(this).value)
    override operator fun minus(other: Duration) = clone(value - other.to(this).value)
    override operator fun times(other: Number) = clone(value * other.toDouble())
    override operator fun div(other: Number) = clone(value / other.toDouble())
    override operator fun unaryMinus() = clone(-value)

    override fun toString(): String = "$value ${unitToString()}" + if (value != 1.0) "s" else ""
    override fun unitToString(): String = "minute"
}