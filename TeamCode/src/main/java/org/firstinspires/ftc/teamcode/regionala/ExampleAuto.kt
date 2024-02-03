package org.firstinspires.ftc.teamcode.regionala

import org.firstinspires.ftc.teamcode.lib.opmode.MultiThreadOpMode
import org.firstinspires.ftc.teamcode.robot.MotorEx.Companion.rev12to1
import org.firstinspires.ftc.teamcode.robot.ServoEx.Companion.axonMax355

class ExampleAuto : MultiThreadOpMode() {
    override fun sideRunOpMode() {

    }

    override fun mainRunOpMode() {
        val motor = hardwareMap.rev12to1("leftFront")
        val servo = hardwareMap.axonMax355("intake")
    }
}