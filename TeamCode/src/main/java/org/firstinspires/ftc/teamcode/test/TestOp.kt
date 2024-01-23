package org.firstinspires.ftc.teamcode.test

import org.firstinspires.ftc.teamcode.lib.opmode.OpModeEx

class TestOp : OpModeEx() {
    private val robot by opModeLazy {
        val rob = Robot(hardwareMap)
        registerFeature(rob)
        rob
    }


    override fun initEx() {

    }

    override fun loopEx() {
    }
}