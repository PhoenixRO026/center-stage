package org.firstinspires.ftc.teamcode.lib.opmode

import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Photon
abstract class MultiThreadOpMode : LinearOpMode() {

    override fun runOpMode() {
        val secondThread = Thread(::sideRunOpMode)
        secondThread.start()
        mainRunOpMode()
    }

    abstract fun sideRunOpMode()

    abstract fun mainRunOpMode()
}