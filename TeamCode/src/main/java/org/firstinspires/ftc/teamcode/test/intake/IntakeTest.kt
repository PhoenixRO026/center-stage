package org.firstinspires.ftc.teamcode.test.intake

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.lib.opmode.OpModeEx

class IntakeTest : OpModeEx() {
    val motor by opModeLazy {
        hardwareMap.get(DcMotorEx::class.java, "intake")
    }

    val servo by opModeLazy {
        hardwareMap.get(Servo::class.java, "intakeServo")
    }

    var servoPos: Double = 0.0
        set(value) {
            servo.position = value.coerceIn(0.0, 1.0).scaleTo(0.489, 0.783)
            field = value.coerceIn(0.0, 1.0)
        }

    override fun initEx() {
        servoPos = 0.0
    }

    override fun loopEx() {

    }

    fun Double.scaleTo(min: Double, max: Double): Double {
        return min + this * (max - min)
    }
}