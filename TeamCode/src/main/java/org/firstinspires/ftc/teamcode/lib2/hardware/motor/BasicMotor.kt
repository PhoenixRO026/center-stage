package org.firstinspires.ftc.teamcode.lib2.hardware.motor

import com.qualcomm.hardware.lynx.LynxDcMotorController
import com.qualcomm.robotcore.hardware.DcMotorImplEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction

class BasicMotor(
    dcMotor: DcMotorSimple,
    direction: Direction
) {
    @JvmField val innerMotor = dcMotor as DcMotorImplEx
    @JvmField val controller = innerMotor.controller as LynxDcMotorController

    var power: Double = 0.0
        set(value) {
            if (field != value) {
                innerMotor.power = value
            }
            field = value
        }

    var direction = direction
        set(value) {
            if (field != value) {
                innerMotor.direction = value
            }
            field = value
        }

    init {
        innerMotor.resetDeviceConfigurationForOpMode()
        innerMotor.direction = direction
    }
}