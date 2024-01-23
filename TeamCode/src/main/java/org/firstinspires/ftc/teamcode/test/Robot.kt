package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.lib.opmode.TimeListener
import org.firstinspires.ftc.teamcode.lib.opmode.TimeOpModeImpl
import org.firstinspires.ftc.teamcode.lib.opmode.Feature

class Robot(hardwareMap: HardwareMap) : Feature, TimeListener by TimeOpModeImpl() {

}