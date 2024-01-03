package com.phoenix_ro026.phoenixlib.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl
import com.qualcomm.robotcore.hardware.HardwareMap

abstract class OpModeEx: LinearOpMode() {
    override fun runOpMode() {

    }
    abstract fun initEx()

    //fun init_loop()
}