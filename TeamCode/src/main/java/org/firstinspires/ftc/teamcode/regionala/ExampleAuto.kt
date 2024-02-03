package org.firstinspires.ftc.teamcode.regionala

import org.firstinspires.ftc.teamcode.lib.opmode.MultiThreadOpMode
import org.firstinspires.ftc.teamcode.robot.MotorEx.Companion.rev12to1

class ExampleAuto : MultiThreadOpMode() {
    override fun sideRunOpMode() {

    }

    override fun mainRunOpMode() {
        val motor = hardwareMap.rev12to1("leftFront")
    }
}