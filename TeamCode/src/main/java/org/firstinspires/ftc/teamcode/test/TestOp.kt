package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.lib.opmode.OpModeEx

@TeleOp
class TestOp : OpModeEx() {
    private val robot by opModeLazy {
        Robot(hardwareMap)
    }


    override fun initEx() {

    }

    override fun loopEx() {
    }
}