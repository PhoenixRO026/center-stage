package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.lib.opmode.MultiThreadOpMode
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.ms
import org.firstinspires.ftc.teamcode.lib.units.s

@Disabled
@TeleOp
class ColorTest2 : MultiThreadOpMode() {
    override fun sideRunOpMode() {
        val colorV2 = hardwareMap.get(ColorRangeSensor::class.java, "colorV2")

        var deltaTime: Time
        var previousTime = System.currentTimeMillis().ms

        waitForStart()

        while (opModeIsActive()) {
            val now = System.currentTimeMillis().ms
            deltaTime = now - previousTime
            previousTime = now

            telemetry.addData("v2 red", colorV2.normalizedColors.red)
            telemetry.addData("v2 green", colorV2.normalizedColors.green)
            telemetry.addData("v2 blue", colorV2.normalizedColors.blue)
            telemetry.addData("v2 lightDetected", colorV2.lightDetected)
            telemetry.addData("v2 distance mm", colorV2.getDistance(DistanceUnit.MM))

            telemetry.addData("side fps", 1.s / deltaTime)

            telemetry.update()
        }
    }

    override fun mainRunOpMode() {
        val colorV3 = hardwareMap.get(ColorRangeSensor::class.java, "colorV3")

        var deltaTime: Time
        var previousTime = System.currentTimeMillis().ms

        waitForStart()

        while (opModeIsActive()) {
            val now = System.currentTimeMillis().ms
            deltaTime = now - previousTime
            previousTime = now

            telemetry.addData("v3 red", colorV3.normalizedColors.red)
            telemetry.addData("v3 green", colorV3.normalizedColors.green)
            telemetry.addData("v3 blue", colorV3.normalizedColors.blue)
            telemetry.addData("v3 lightDetected", colorV3.lightDetected)
            telemetry.addData("v3 distance mm", colorV3.getDistance(DistanceUnit.MM))

            telemetry.addData("main fps", 1.s / deltaTime)

            telemetry.update()
        }
    }
}