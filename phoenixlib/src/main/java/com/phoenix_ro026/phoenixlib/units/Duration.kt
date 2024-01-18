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

operator fun <T: Duration> T.plus(other: Duration) = clone(value + other.to(this).value)
operator fun <T: Duration> T.minus(other: Duration) = clone(value - other.to(this).value)
operator fun <T: Duration> T.times(other: Number) = clone(value * other.toDouble())
operator fun <T: Duration> T.div(other: Number) = clone(value / other.toDouble())
operator fun <T: Duration> T.unaryMinus() = clone(-value)

@JvmField
val SEC= 1.sec
val Number.sec get() = Seconds(toDouble())
fun sec(number: Double) = number.sec
class Seconds(override val value: Double) : Duration {
    override fun toSeconds(): Seconds = this
    override fun toMilliseconds(): Milliseconds = Milliseconds(value * 1000.0)
    override fun toMinutes(): Minutes = Minutes(value / 60.0)

    /*override operator fun plus(other: Duration) : Seconds = super.plus(other) as Seconds
    override operator fun minus(other: Duration) : Seconds = super.minus(other) as Seconds
    override operator fun times(other: Number) : Seconds = super.times(other) as Seconds
    override operator fun div(other: Number) : Seconds = super.div(other) as Seconds*/

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

    /*override operator fun plus(other: Duration) : Milliseconds = super.plus(other) as Milliseconds
    override operator fun minus(other: Duration) : Milliseconds = super.minus(other) as Milliseconds
    override operator fun times(other: Number) : Milliseconds = super.times(other) as Milliseconds
    override operator fun div(other: Number) : Milliseconds = super.div(other) as Milliseconds*/

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

    /*override operator fun plus(other: Duration) : Minutes = super.plus(other) as Minutes
    override operator fun minus(other: Duration) : Minutes = super.minus(other) as Minutes
    override operator fun times(other: Number) : Minutes = super.times(other) as Minutes
    override operator fun div(other: Number) : Minutes = super.div(other) as Minutes*/

    override fun toString(): String = "$value ${unitToString()}" + if (value != 1.0) "s" else ""
    override fun unitToString(): String = "minute"
}