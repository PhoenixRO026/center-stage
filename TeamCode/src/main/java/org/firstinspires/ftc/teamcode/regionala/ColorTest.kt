package org.firstinspires.ftc.teamcode.regionala

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.hardware.configuration.LynxConstants
import org.firstinspires.ftc.teamcode.lib.opmode.MultiThreadOpMode
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.ms
import org.firstinspires.ftc.teamcode.lib.units.s
import org.firstinspires.ftc.teamcode.robot.ColorSensorsMulti.Companion.colorSensMulti

@Disabled
@Autonomous
class ColorTest : MultiThreadOpMode() {
    private val colorSensors by opModeLazy {
        hardwareMap.colorSensMulti()
    }

    override fun sideRunOpMode() {
    }

    override fun mainRunOpMode() {
        var previousTime = System.currentTimeMillis().ms
        var deltaTime: Time

        val expansionHub = hardwareMap.getAll(LynxModule::class.java).first {
            it.isParent && !LynxConstants.isEmbeddedSerialNumber(it.serialNumber)
        }

        expansionHub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL

        val dash = FtcDashboard.getInstance()

        telemetry = MultipleTelemetry(telemetry, dash.telemetry)

        waitForStart()

        while (isStarted && !isStopRequested) {
            val now = System.currentTimeMillis().ms
            deltaTime = now - previousTime
            previousTime = now

            expansionHub.clearBulkCache()

            colorSensors.read()

            telemetry.addData("main delta fps", 1.s / deltaTime)
            telemetry.addData("left red", colorSensors.leftColor.red)
            telemetry.addData("left green", colorSensors.leftColor.green)
            telemetry.addData("left blue", colorSensors.leftColor.blue)
            telemetry.addData("left alpha", colorSensors.leftColor.alpha)
//            telemetry.addData("left distance mm", colorSensors.leftDistance.mm)
//            telemetry.addData("left light", colorSensors.leftLight)
            telemetry.addData("left pixel in", colorSensors.leftColor.alpha > 0.75)
            telemetry.addData("right red", colorSensors.rightColor.red)
            telemetry.addData("right green", colorSensors.rightColor.green)
            telemetry.addData("right blue", colorSensors.rightColor.blue)
            telemetry.addData("right alpha", colorSensors.rightColor.alpha)
//            telemetry.addData("right distance mm", colorSensors.rightDistance.mm)
//            telemetry.addData("right light", colorSensors.rightLight)
            telemetry.addData("right pixel in", colorSensors.rightColor.alpha > 0.045)
            telemetry.update()
        }
    }
}