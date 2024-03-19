package org.firstinspires.ftc.teamcode.lib.controller.utils

import java.util.Stack

class SizedStack<T>(
    val maxSize: Int = 1,
    defaultItem: T? = null
) : Stack<T>() {
    init {
        if (defaultItem != null) {
            while (size < maxSize) {
                push(defaultItem)
            }
        }
    }

    override fun push(item: T): T {
        while (size >= maxSize) {
            pop()
        }
        return super.push(item)
    }
}

fun SizedStack<Double>.doubleArray(): DoubleArray {
    val array = DoubleArray(maxSize)
    for (i in indices) {
        array[i] = get(i)
    }
    return array
}