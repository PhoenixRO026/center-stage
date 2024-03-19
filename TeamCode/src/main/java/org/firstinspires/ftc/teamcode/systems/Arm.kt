package org.firstinspires.ftc.teamcode.systems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.lib.hardware.servo.RangedServo.Companion.rangedServo
import org.firstinspires.ftc.teamcode.lib.hardware.servo.RangedSpeedServo.Companion.rangedSpeedServo

class Arm(
    hardwareMap: HardwareMap
) {
    companion object {
        fun HardwareMap.arm() = Arm(this)
    }

    @Config
    data object ArmConfig {
        @JvmField var armServoOffset: Double = 0.02
        @JvmField var armSpeed: Double = 0.8
        @JvmField var intakePos: Double = 0.666
        @JvmField var scorePos: Double = 0.251
    }

    private val leftRange = 0.0..(1.0 - ArmConfig.armServoOffset)
    private val rightRange = ArmConfig.armServoOffset..1.0

    private val leftServo = hardwareMap.rangedServo("leftArm", range = leftRange)
    private val rightServo = hardwareMap.rangedSpeedServo(
        deviceName = "rightArm",
        speed = ArmConfig.armSpeed,
        range = rightRange,
        coupledServos = listOf(leftServo)
    )

    var position: Double by rightServo::position

    var targetPosition: Double by rightServo::targetPosition

    val isBusy by rightServo::isBusy

    fun intakePos() {
        targetPosition = ArmConfig.intakePos
    }

    fun scorePos() {
        targetPosition = ArmConfig.scorePos
    }

    fun update() {
        rightServo.update()
    }

    init {
        position = ArmConfig.intakePos
    }
}