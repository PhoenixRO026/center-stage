package org.firstinspires.ftc.teamcode.systems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.lib.hardware.servo.RangedServo.Companion.rangedServo
import org.firstinspires.ftc.teamcode.lib.hardware.servo.RangedSpeedServo.Companion.rangedSpeedServo

class Box(
    hardwareMap: HardwareMap,
) {
    @Config
    data object BoxConfig {
        @JvmField var boxServoOffset: Double = 0.025
        @JvmField var bosSpeed = 0.1
    }

    private val leftServoRange = BoxConfig.boxServoOffset..1.0
    private val rightServoRange = 0.0..(1.0 - BoxConfig.boxServoOffset)

    private val leftAngleServo = hardwareMap.rangedServo("leftBox", Servo.Direction.REVERSE, leftServoRange)
    private val rightAngleServo = hardwareMap.rangedSpeedServo(
        deviceName = "rightBox",
        speed = BoxConfig.bosSpeed,
        range = rightServoRange,
        coupledServos = listOf(leftAngleServo)
    )

    var position: Double by rightAngleServo::position

    var targetPosition: Double by rightAngleServo::targetPosition


}