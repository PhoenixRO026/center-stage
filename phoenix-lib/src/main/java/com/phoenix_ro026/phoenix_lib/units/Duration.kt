package com.phoenix_ro026.phoenix_lib.units

sealed interface Duration {
    val value: Double
    @Suppress("UNCHECKED_CAST")
    fun <T: Duration> to(unit: T): T {
        @Suppress("USELESS_CAST")
        return when(unit as Duration) {
            is Milisecond -> toMiliseconds() as T
            is Minute -> toMinutes() as T
            is Second -> toSeconds() as T
        }
    }
    operator fun plus(other: Duration) = clone(value + other.to(this).value)
    operator fun minus(other: Duration) = clone(value - other.to(this).value)
    operator fun times(other: Number) = clone(value * other.toDouble())
    operator fun div(other: Number) = clone(value / other.toDouble())
    fun toSeconds(): Second
    fun toMiliseconds(): Milisecond
    fun toMinutes(): Minute
}

@Suppress("UNCHECKED_CAST")
private fun <T: Duration> T.clone(newValue: Double): T {
    @Suppress("USELESS_CAST")
    return when(this as Duration) {
        is Milisecond -> Milisecond(newValue) as T
        is Minute -> Minute(newValue) as T
        is Second -> Second(newValue) as T
    }
}

@JvmField
val SEC = 1.sec
val Number.sec get() = Second(toDouble())
fun sec(number: Double) = number.sec
class Second(override val value: Double) : Duration {
    override fun toSeconds(): Second = this
    override fun toMiliseconds(): Milisecond = Milisecond(value * 1000.0)
    override fun toMinutes(): Minute = Minute(value / 60.0)
    override operator fun plus(other: Duration): Second {
        return super.plus(other) as Second
    }
    override operator fun minus(other: Duration): Second {
        return super.minus(other) as Second
    }
    override operator fun times(other: Number): Second {
        return super.times(other) as Second
    }
    override operator fun div(other: Number): Second {
        return super.div(other) as Second
    }
}

@JvmField
val MILISEC = 1.milisec
val Number.milisec get() = Milisecond(toDouble())
fun milisec(number: Double) = number.milisec
class Milisecond(override val value: Double) : Duration {
    override fun toSeconds(): Second = Second(value / 1000.0)
    override fun toMiliseconds(): Milisecond = this
    override fun toMinutes(): Minute = Minute((value / 1000.0) / 60.0)
    override operator fun plus(other: Duration): Milisecond {
        return super.plus(other) as Milisecond
    }
    override operator fun minus(other: Duration): Milisecond {
        return super.minus(other) as Milisecond
    }
    override operator fun times(other: Number): Milisecond {
        return super.times(other) as Milisecond
    }
    override operator fun div(other: Number): Milisecond {
        return super.div(other) as Milisecond
    }
}

@JvmField
val MIN = 1.min
val Number.min get() = Minute(toDouble())
fun min(number: Double) = number.min
class Minute(override val value: Double) : Duration {
    override fun toSeconds(): Second = Second(value * 60.0)
    override fun toMiliseconds(): Milisecond = Milisecond(value * 60.0 * 1000.0)
    override fun toMinutes(): Minute = this
    override operator fun plus(other: Duration): Minute {
        return super.plus(other) as Minute
    }
    override operator fun minus(other: Duration): Minute {
        return super.minus(other) as Minute
    }
    override operator fun times(other: Number): Minute {
        return super.times(other) as Minute
    }
    override operator fun div(other: Number): Minute {
        return super.div(other) as Minute
    }
}