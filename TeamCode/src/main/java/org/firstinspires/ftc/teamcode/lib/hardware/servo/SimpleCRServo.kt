package org.firstinspires.ftc.teamcode.lib.hardware.servo

import com.qualcomm.hardware.lynx.LynxServoController
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl

class SimpleCRServo @JvmOverloads constructor(
    crservo: CRServo,
    direction: Direction = Direction.FORWARD
) {
    companion object {
        fun HardwareMap.simpleCRServo(
            deviceName: String,
            direction: Direction = Direction.FORWARD
        ) = SimpleCRServo(this, deviceName, direction)
    }

    @JvmOverloads constructor(
        hardwareMap: HardwareMap,
        deviceName: String,
        direction: Direction = Direction.FORWARD
    ) : this(hardwareMap.get(CRServo::class.java, deviceName), direction)

    @JvmField val innerCRServo = crservo as CRServoImplEx
    @JvmField val controller = crservo.controller as LynxServoController

    var power: Double = 0.0
        set(value) {
            val newValue = value.coerceIn(-1.0, 1.0)
            if (field != newValue) {
                innerCRServo.power = newValue
            }
            field = newValue
        }

    var direction: Direction = direction
        set(value) {
            if (field != value) {
                innerCRServo.direction = value
            }
            field = value
        }

    var pwmRange: PwmControl.PwmRange = innerCRServo.pwmRange
        set(value) {
            if (field != value) {
                innerCRServo.pwmRange = value
            }
            field = value
        }

    init {
        innerCRServo.resetDeviceConfigurationForOpMode()
        innerCRServo.direction = direction
    }
}