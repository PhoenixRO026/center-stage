package org.firstinspires.ftc.teamcode.lib.opmode

import java.lang.RuntimeException
import kotlin.reflect.KProperty

class OpModeLazyImpl<T>(private val constructor: () -> T) : OpModeLazy<T> {
    private var _value: T? = null

    override fun getValue(thisRef: Any, property: KProperty<*>): T {
        _value?.let {
            return it
        }
        throw RuntimeException("Used OpModeLazy value before init")
    }

    override fun init() {
        _value = constructor()
    }
}