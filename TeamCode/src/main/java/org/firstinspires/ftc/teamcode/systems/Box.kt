package org.firstinspires.ftc.teamcode.systems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.lib.hardware.servo.RangedServo.Companion.rangedServo
import org.firstinspires.ftc.teamcode.lib.hardware.servo.RangedSpeedServo.Companion.rangedSpeedServo
import org.firstinspires.ftc.teamcode.lib.hardware.servo.SimpleCRServo.Companion.simpleCRServo

class Box(
    hardwareMap: HardwareMap
) {
    companion object {
        fun HardwareMap.box() = Box(this)
    }

    @Config
    data object BoxConfig {
        @JvmField var boxServoOffset: Double = 0.08
        @JvmField var boxSpeed = 0.8
        @JvmField var intakePos = 0.9337
        @JvmField var scorePos = 0.4989

        @JvmField var intakeToScoreTravelWaitSec = Arm.ArmConfig.intakeToScoreTravelWaitSec

        @JvmField var ejectOneWhitePixelWaitSec = 0.25
        @JvmField var ejectYellowPixelWaitSec = 0.45
        @JvmField var ejectTwoPixelWaitSec = 0.6
    }

    private val leftServoRange = BoxConfig.boxServoOffset..1.0
    private val rightServoRange = 0.0..(1.0 - BoxConfig.boxServoOffset)

    private val leftAngleServo = hardwareMap.rangedServo("leftBox", Servo.Direction.REVERSE, leftServoRange)
    private val rightAngleServo = hardwareMap.rangedSpeedServo(
        deviceName = "rightBox",
        speed = BoxConfig.boxSpeed,
        range = rightServoRange,
        coupledServos = listOf(leftAngleServo)
    )
    private val wheel = hardwareMap.simpleCRServo("wheelBox", DcMotorSimple.Direction.REVERSE)

    var position: Double by rightAngleServo::position

    var targetPosition: Double by rightAngleServo::targetPosition

    var power: Double by wheel::power

    val isBusy by rightAngleServo::isBusy

    fun intakePos() {
        targetPosition = BoxConfig.intakePos
    }

    fun scorePos() {
        targetPosition = BoxConfig.scorePos
    }

    fun update() {
        rightAngleServo.update()
    }

    init {
        position = BoxConfig.intakePos
    }
}