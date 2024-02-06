package org.firstinspires.ftc.teamcode.robot.hardware

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.configuration.LynxConstants

fun HardwareMap.controlHub() = getAll(LynxModule::class.java).first {
    it.isParent && LynxConstants.isEmbeddedSerialNumber(it.serialNumber)
}

fun HardwareMap.expansionHub() = getAll(LynxModule::class.java).first {
    it.isParent && !LynxConstants.isEmbeddedSerialNumber(it.serialNumber)
}