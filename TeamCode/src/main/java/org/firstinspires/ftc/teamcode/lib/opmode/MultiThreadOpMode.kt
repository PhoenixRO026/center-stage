package org.firstinspires.ftc.teamcode.lib.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

abstract class MultiThreadOpMode : LinearOpMode() {
    private val lazyInits: MutableList<OpModeInit> = mutableListOf()

    fun <T> opModeLazy(constructor: () -> T): OpModeLazy<T> {
        val thing = OpModeLazyImpl {
            constructor()
        }
        lazyInits.add(thing)
        return thing
    }

    final override fun runOpMode() {
        lazyInits.forEach { it.init() }
        val secondThread = Thread(::sideRunOpMode)
        secondThread.start()
        mainRunOpMode()
        secondThread.interrupt()
    }

    abstract fun sideRunOpMode()

    abstract fun mainRunOpMode()

}