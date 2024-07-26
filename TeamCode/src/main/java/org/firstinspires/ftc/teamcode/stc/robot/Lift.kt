package org.firstinspires.ftc.teamcode.stc.robot

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.stc.PIDController
import kotlin.math.abs

class Lift (hardwareMap: HardwareMap) {
    companion object {
        const val hangTics = -600

        val controller = PIDController(
            kP = 0.008,
            kI = 0.005,
            kD = 0.0008
        )
        val toleranceTicks = 16
        val kF = 0.16
    }

    val leftLiftMotor: DcMotor = hardwareMap.get(DcMotor::class.java, "leftLift")
    val rightLiftMotor: DcMotor = hardwareMap.get(DcMotor::class.java, "rightLift")

    enum class Mode {
        POWER,
        TARGET
    }

    private var mode = Mode.POWER
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

    val busy get() = abs(leftLiftMotor.currentPosition - targetPositionTicks) > toleranceTicks

    fun hang(){
        if (hanging) return

        targetPositionTicks = leftLiftMotor.currentPosition + hangTics

        mode = Mode.TARGET

        hanging = true
    }

    fun unhang(){
        if (!hanging) return

        mode = Mode.POWER

        leftLiftMotor.power = 0.0
        rightLiftMotor.power = 0.0

        hanging = false
    }

    var power : Number
        get() = leftLiftMotor.power
        set(value){
            if (mode != Mode.POWER) return

            leftLiftMotor.power = value.toDouble()
            rightLiftMotor.power = value.toDouble()
        }

    fun update() {
        val feedback = controller.calculate(leftLiftMotor.currentPosition.toDouble(), targetPositionTicks.toDouble()) + kF

        if (mode == Mode.TARGET) {
            leftLiftMotor.power = feedback
            rightLiftMotor.power = feedback
        }
    }
}