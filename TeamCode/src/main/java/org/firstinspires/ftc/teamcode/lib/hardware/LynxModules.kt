@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.lib.hardware

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.configuration.LynxConstants

fun HardwareMap.controlHub(): LynxModule = getAll(LynxModule::class.java).first {
    it.isParent && LynxConstants.isEmbeddedSerialNumber(it.serialNumber)
}

fun HardwareMap.expansionHub(): LynxModule = getAll(LynxModule::class.java).first {
    it.isParent && !LynxConstants.isEmbeddedSerialNumber(it.serialNumber)
}