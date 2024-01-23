package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.lib.opmode.DeltaTimeListener
import org.firstinspires.ftc.teamcode.lib.opmode.DeltaTimeOpModeImpl
import org.firstinspires.ftc.teamcode.lib.opmode.Feature

class Robot(hardwareMap: HardwareMap) : Feature, DeltaTimeListener by DeltaTimeOpModeImpl() {

}