package org.firstinspires.ftc.teamcode.lib.multi

import java.util.Queue
import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.atomic.AtomicReference
import kotlin.reflect.KProperty

//TODO: needs testing
class LazyWrite<T: Any>(
    private val queue: Queue<Task>,
    private val writeT: (T) -> Unit,
    private val getValueT: () -> T
) {
    data class Task(
        val time: Long = System.nanoTime(),
        val task: () -> Unit
    ) {
        override fun equals(other: Any?): Boolean {
            return if (other is Task) {
                time == other.time
            } else super.equals(other)
        }

        override fun hashCode(): Int {
            return time.hashCode()
        }
    }

    private var previousTask: Task? = null

    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: T) {
        previousTask?.let { task ->
            queue.remove(task)
        }
        val newTask = Task {
            writeT(value)
        }
        previousTask = newTask
        queue.add(newTask)
    }

    operator fun getValue(thisRef: Any, property: KProperty<*>) = getValueT()
}