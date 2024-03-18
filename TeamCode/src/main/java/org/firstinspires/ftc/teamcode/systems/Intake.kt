package org.firstinspires.ftc.teamcode.systems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.lib.hardware.motor.SimpleMotor.Companion.simpleMotor
import org.firstinspires.ftc.teamcode.lib.hardware.servo.RangedSpeedServo.Companion.rangedSpeedServo

class Intake(
    hardwareMap: HardwareMap
) {
    companion object {
        fun HardwareMap.intake() = Intake(this)
    }

    @Config
    data object IntakeConfig {
        @JvmField var rangeStart = 0.0
        @JvmField var rangeEnd = 0.5
        @JvmField var speed = 0.2

        @JvmField var groundPos = 0.1
        @JvmField var initPos = groundPos
    }

    private val servoRange = IntakeConfig.rangeStart..IntakeConfig.rangeEnd

    val angleServo = hardwareMap.rangedSpeedServo(
        deviceName =  "intakeAngle",
        direction = Servo.Direction.REVERSE,
        speed =  IntakeConfig.speed,
        range = servoRange
    )
    private val motor = hardwareMap.simpleMotor("intake")

    var position by angleServo::position

    var targetPosition by angleServo::targetPosition

    var power by motor::power

    val isBusy by angleServo::isBusy

    fun update() {
        angleServo.update()
    }

    init {
        position = IntakeConfig.initPos
    }
}