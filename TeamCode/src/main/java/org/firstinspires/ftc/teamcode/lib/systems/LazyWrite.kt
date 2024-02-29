package org.firstinspires.ftc.teamcode.lib.systems

import java.util.Queue
import kotlin.reflect.KProperty

class LazyWrite<T: Any>(
    private val queue: Queue<() -> Unit>,
    private val writeT: (T) -> Unit,
    private val getValueT: () -> T
) {
    @Volatile
    private var waitingOnQueue = false

    private var cachedValue: T = getValueT()

    private fun getCache() = cachedValue

    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: T) {
        cachedValue = value
        if (!waitingOnQueue) {
            waitingOnQueue = true
            queue.add {
                writeT(getCache())
                waitingOnQueue = false
            }
        }
    }

    operator fun getValue(thisRef: Any, property: KProperty<*>) = getValueT()
}