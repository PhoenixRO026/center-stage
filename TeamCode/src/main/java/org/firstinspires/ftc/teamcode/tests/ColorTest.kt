package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.systems.multi.Color2Multi.Companion.color2Multi

@Disabled
@TeleOp
class ColorTest : LinearOpMode() {
    override fun runOpMode() {
        val color = hardwareMap.color2Multi()

        color.readingEnabled = true

        waitForStart()

        while (opModeIsActive()) {
            color.read()

            telemetry.addData("red", color.color.red)
            telemetry.addData("green", color.color.green)
            telemetry.addData("blue", color.color.blue)
            telemetry.addData("alpha", color.color.alpha)
            telemetry.addData("yellowIn", color.yellowPixelIn)
            telemetry.update()
        }
    }
}