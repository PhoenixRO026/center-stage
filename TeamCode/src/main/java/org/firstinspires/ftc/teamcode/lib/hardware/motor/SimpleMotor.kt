package org.firstinspires.ftc.teamcode.lib.hardware.motor

import com.qualcomm.hardware.lynx.LynxDcMotorController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorImplEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction

class SimpleMotor(
    dcMotor: DcMotorSimple,
    direction: Direction = Direction.FORWARD
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

    var mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        set(value) {
            if (field != value) {
                innerMotor.mode = value
            }
            field = value
        }

    init {
        innerMotor.resetDeviceConfigurationForOpMode()
        innerMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        innerMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        innerMotor.direction = direction
        innerMotor.power = 0.0
    }
}