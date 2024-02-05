package org.firstinspires.ftc.teamcode.robot

import kotlin.reflect.KProperty

class LazyWrite<T: Any>(
    private val writeT: (T) -> Unit,
    private val getValueT: () -> T
) {
    private var writeCahce = false

    private lateinit var cachedValue: T

    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: T) {
        cachedValue = value
        writeCahce = true
    }

    fun write() {
        if (writeCahce) {
            writeT(cachedValue)
            writeCahce = false
        }
    }

    operator fun getValue(thisRef: Any, property: KProperty<*>) = getValueT()
}