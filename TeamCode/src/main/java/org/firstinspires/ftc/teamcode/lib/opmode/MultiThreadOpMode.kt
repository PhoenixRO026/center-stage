package org.firstinspires.ftc.teamcode.lib.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

abstract class MultiThreadOpMode : LinearOpMode() {

    final override fun runOpMode() {
        val secondThread = Thread(::sideRunOpMode)
        secondThread.start()
        mainRunOpMode()
        secondThread.interrupt()
    }

    abstract fun sideRunOpMode()

    abstract fun mainRunOpMode()

}