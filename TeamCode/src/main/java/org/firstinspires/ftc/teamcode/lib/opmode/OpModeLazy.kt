package org.firstinspires.ftc.teamcode.lib.opmode

import kotlin.reflect.KProperty

interface OpModeLazy<T> : OpModeInit {
    operator fun getValue(thisRef: Any, property: KProperty<*>): T
}