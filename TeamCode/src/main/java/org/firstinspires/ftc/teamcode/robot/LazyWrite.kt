package org.firstinspires.ftc.teamcode.robot

import kotlin.reflect.KProperty

class LazyWrite<T: Any>(
    private val writeT: (T) -> Unit,
    private val getValueT: () -> T
) {
    @Volatile
    private var cachedValue: T? = null

    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: T) {
        cachedValue = value
    }

    fun write() {
        if (cachedValue != null) {
            cachedValue?.let {
                writeT(it)
            }
            cachedValue = null
        }
    }

    operator fun getValue(thisRef: Any, property: KProperty<*>) = getValueT()
}