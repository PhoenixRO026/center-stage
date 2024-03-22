package org.firstinspires.ftc.teamcode.systems.multi

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.ms
import org.firstinspires.ftc.teamcode.lib.units.s

class ColorMulti(
    hardwareMap: HardwareMap
) {
    companion object {
        fun HardwareMap.colorMulti() = ColorMulti(this)
    }

    @Config
    data object ColorConfig {
        @JvmField var backAlphaThreshold = 0.75
        @JvmField var frontAlphaThreshold = 0.045
        @JvmField var maxPixelWaitSec = 0.8
    }

    //private val backColorSens = hardwareMap.get(ColorRangeSensor::class.java, "backColor")
    //private val frontColorSens = hardwareMap.get(ColorRangeSensor::class.java, "frontColor")

    val backColorIn get() = false

    val frontColorIn get() = false

    private var freshRead = false

    private var readingEnabled = false

    //private var innerBackColor: NormalizedRGBA = backColorSens.normalizedColors

    //private var innerFrontColor: NormalizedRGBA = frontColorSens.normalizedColors

    //val backColor by ::innerBackColor

    //val frontColor by ::innerFrontColor

    fun waitForPixels() = object : Action {
        var init = true
        var startTime = 0.ms
        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false
                freshRead = false
                readingEnabled = true
                startTime = Time.now()
            }

            if ((freshRead && backColorIn && frontColorIn)
                    || Time.now() - startTime > ColorConfig.maxPixelWaitSec.s) {
                readingEnabled = false
                return false
            }

            return true
        }
    }

    fun waitForFrontPixel() = object : Action {
        var init = true
        var startTime = 0.ms
        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false
                freshRead = false
                readingEnabled = true
                startTime = Time.now()
            }

            if (/*(freshRead && frontColorIn)
                || */Time.now() - startTime > ColorConfig.maxPixelWaitSec.s) {
                readingEnabled = false
                return false
            }

            return true
        }
    }

    fun read() {
        if (readingEnabled) {
            freshRead = true
            //innerBackColor = backColorSens.normalizedColors
            //innerFrontColor = frontColorSens.normalizedColors
        }
    }
}