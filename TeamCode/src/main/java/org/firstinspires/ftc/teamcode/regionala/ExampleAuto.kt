package org.firstinspires.ftc.teamcode.regionala

import org.firstinspires.ftc.teamcode.lib.opmode.MultiThreadOpMode
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.ms
import org.firstinspires.ftc.teamcode.robot.ArmMulti.Companion.armMulti
import org.firstinspires.ftc.teamcode.robot.ClawMulti.Companion.clawMulti

class ExampleAuto : MultiThreadOpMode() {
    private val claw by opModeLazy {
        hardwareMap.clawMulti()
    }

    private val arm by opModeLazy {
        hardwareMap.armMulti()
    }

    override fun sideRunOpMode() {
        var previousTime = System.currentTimeMillis().ms
        var deltaTime: Time

        waitForStart()

        while (opModeIsActive()) {
            val now = System.currentTimeMillis().ms
            deltaTime = now - previousTime
            previousTime = now

            claw.write()
            arm.write()
            arm.update(deltaTime)
        }
    }

    override fun mainRunOpMode() {
        var previousTime = System.currentTimeMillis().ms
        var deltaTime: Time

        waitForStart()

        while (opModeIsActive()) {
            val now = System.currentTimeMillis().ms
            deltaTime = now - previousTime
            previousTime = now

            claw.clawPosition = 0.0
            arm.position = 0.0
        }
    }
}