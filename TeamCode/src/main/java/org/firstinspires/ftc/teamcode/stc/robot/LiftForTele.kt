package org.firstinspires.ftc.teamcode.stc.robot

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class LiftForTele (hardwareMap: HardwareMap) {
    companion object {
        const val hangTics = -300
        const val hangPow = 0.5
    }

    val leftLiftMotor = hardwareMap.get(DcMotor::class.java, "leftLift")
    val rightLiftMotor = hardwareMap.get(DcMotor::class.java, "rightLift")

    var hanging = false

    init {
        leftLiftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightLiftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        leftLiftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightLiftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    fun hang(){
        if(hanging) return

        leftLiftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightLiftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        leftLiftMotor.targetPosition = hangTics
        rightLiftMotor.targetPosition = hangTics

        leftLiftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        rightLiftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION

        leftLiftMotor.power = hangPow
        rightLiftMotor.power = hangPow

        hanging = true
    }

    fun unhang(){
        if(!hanging) return
        leftLiftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightLiftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        leftLiftMotor.power = 0.0
        rightLiftMotor.power = 0.0

        hanging = false
    }
    var power : Number
        get() = leftLiftMotor.power
        set(value){
            if(hanging) return

            leftLiftMotor.power = value.toDouble()
            rightLiftMotor.power = value.toDouble()
        }
}