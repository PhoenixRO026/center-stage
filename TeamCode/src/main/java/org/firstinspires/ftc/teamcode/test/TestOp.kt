package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.lib.opmode.OpModeEx

@Disabled
@TeleOp
class TestOp : OpModeEx() {
    private val robot by opModeLazy {
        val rob = Robot(hardwareMap, opModeTime)
        registerFeature(rob)
    }

    override fun initEx() {
    }

    override fun loopEx() {
        telemetry.addData("fps", fps)
        telemetry.addData("delta time", deltaTime)
        telemetry.addData("elapsed time", elapsedTime)
    }
}