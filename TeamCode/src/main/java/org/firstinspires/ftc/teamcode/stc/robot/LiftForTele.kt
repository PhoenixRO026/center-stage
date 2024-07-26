package org.firstinspires.ftc.teamcode.stc.robot

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.stc.PIDController

class LiftForTele (hardwareMap: HardwareMap) {
    companion object {
        const val hangTics = -20
        const val hangPow = 0.1

        @JvmField var controller = PIDController(
            kP = 0.008,
            kI = 0.005,
            kD = 0.0008
        )
        @JvmField var toleranceTicks = 16
        @JvmField var kF = 0.16
    }

    val leftLiftMotor: DcMotor = hardwareMap.get(DcMotor::class.java, "leftLift")
    val rightLiftMotor: DcMotor = hardwareMap.get(DcMotor::class.java, "rightLift")

    private var hanging = false
    private var targetPositionTicks = 0

    init {
        rightLiftMotor.direction = DcMotorSimple.Direction.REVERSE

        leftLiftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightLiftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        leftLiftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightLiftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        leftLiftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightLiftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    fun hang(){
        if (hanging) return

        targetPositionTicks = leftLiftMotor.currentPosition + hangTics

        hanging = true
    }

    fun unhang(){
        if (!hanging) return

        leftLiftMotor.power = 0.0
        rightLiftMotor.power = 0.0

        hanging = false
    }
    var power : Number
        get() = leftLiftMotor.power
        set(value){
            if (hanging) return

            leftLiftMotor.power = value.toDouble()
            rightLiftMotor.power = value.toDouble()
        }

    fun update() {
        val feedback = controller.calculate(leftLiftMotor.currentPosition.toDouble(), targetPositionTicks.toDouble()) + kF

        if (hanging) {
            leftLiftMotor.power = feedback
            rightLiftMotor.power = feedback
        }
    }

}