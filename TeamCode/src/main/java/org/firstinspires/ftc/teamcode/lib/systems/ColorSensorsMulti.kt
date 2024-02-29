package org.firstinspires.ftc.teamcode.lib.systems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.ms
import org.firstinspires.ftc.teamcode.lib.units.s

@Suppress("unused", "MemberVisibilityCanBePrivate")
class ColorSensorsMulti(
    hardwareMap: HardwareMap
) {
    companion object {
        val waitTime = 2.s
        fun colorSensMultiH(hardwareMap: HardwareMap) = ColorSensorsMulti(hardwareMap)

        fun HardwareMap.colorSensMulti() = ColorSensorsMulti(this)
    }

    //0 - V3, 2 - V2
    private val leftColorSens = hardwareMap.get(ColorRangeSensor::class.java, "leftColor")
    private val rightColorSens = hardwareMap.get(ColorRangeSensor::class.java, "rightColor")

    val leftPixelIn get() = leftColor.alpha > 0.75

    val rightPixelIn get() = rightColor.alpha > 0.045

    var leftColor: NormalizedRGBA = leftColorSens.normalizedColors
        private set

    var rightColor: NormalizedRGBA = rightColorSens.normalizedColors
        private set

    fun waitForLeftPixel() = object : Action {
        var init = true
        var startTime: Time = 0.ms

        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false

                startTime = System.currentTimeMillis().ms
            }

            return !leftPixelIn && (System.currentTimeMillis().ms - startTime) < 2.s
        }
    }

    fun waitForRightPixel() = object : Action {
        var init = true
        var startTime: Time = 0.ms

        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false

                startTime = System.currentTimeMillis().ms
            }

            return !rightPixelIn && (System.currentTimeMillis().ms - startTime) < 2.s
        }
    }

//    var leftDistance = leftColorSens.getDistance(DistanceUnit.MM).mm
//        private set

//    var rightDistance = rightColorSens.getDistance(DistanceUnit.MM).mm
//        private set

//    var leftLight = leftColorSens.lightDetected
//        private set

//    var rightLight = rightColorSens.lightDetected
//        private set

    fun read() {
        leftColor = leftColorSens.normalizedColors
        //leftDistance = leftColorSens.getDistance(DistanceUnit.MM).mm
        //leftLight = leftColorSens.lightDetected
        rightColor = rightColorSens.normalizedColors
        //rightDistance = rightColorSens.getDistance(DistanceUnit.MM).mm
        //rightLight = rightColorSens.lightDetected
    }
}