package org.firstinspires.ftc.teamcode.test

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.lib.units.ms
import org.firstinspires.ftc.teamcode.lib.units.s

@Disabled
@TeleOp
class ColorSensorTest : LinearOpMode() {
    override fun runOpMode() {
        val colorV3 = hardwareMap.get(ColorRangeSensor::class.java, "colorV3")
        val colorV2 = hardwareMap.get(ColorRangeSensor::class.java, "colorV2")

        var deltaTime = 20.ms
        var previousTime = System.currentTimeMillis().ms

        val modules = hardwareMap.getAll(LynxModule::class.java)

        modules.forEach {
            it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        waitForStart()



        while (opModeIsActive()) {
            val now = System.currentTimeMillis().ms
            deltaTime = now - previousTime
            previousTime = now

            modules.forEach {
                it.getBulkData()
            }

            telemetry.addData("v3 red", colorV3.normalizedColors.red)
            telemetry.addData("v3 green", colorV3.normalizedColors.green)
            telemetry.addData("v3 blue", colorV3.normalizedColors.blue)
            telemetry.addData("v3 lightDetected", colorV3.lightDetected)
            telemetry.addData("v3 distance mm", colorV3.getDistance(DistanceUnit.MM))

            telemetry.addData("v2 red", colorV2.normalizedColors.red)
            telemetry.addData("v2 green", colorV2.normalizedColors.green)
            telemetry.addData("v2 blue", colorV2.normalizedColors.blue)
            telemetry.addData("v2 lightDetected", colorV2.lightDetected)
            telemetry.addData("v2 distance mm", colorV2.getDistance(DistanceUnit.MM))

            telemetry.addData("fps", 1.s / deltaTime)

            telemetry.update()
        }
    }
}