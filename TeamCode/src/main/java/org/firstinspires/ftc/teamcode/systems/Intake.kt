package org.firstinspires.ftc.teamcode.systems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.lib.hardware.motor.SimpleMotor.Companion.simpleMotor
import org.firstinspires.ftc.teamcode.lib.hardware.servo.RangedSpeedServo.Companion.rangedSpeedServo
import org.firstinspires.ftc.teamcode.lib.units.DeltaTime
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.ms

class Intake(
    hardwareMap: HardwareMap
) {
    companion object {
        fun HardwareMap.intake() = Intake(this)
    }

    @Config
    data object IntakeConfig {
        @JvmField var rangeStart = 0.04
        @JvmField var rangeEnd = 0.35
        @JvmField var speed = 0.2

        @JvmField var groundPos = 0.0
        @JvmField var initPos = groundPos
        @JvmField var aboveStackPose = 0.7
        @JvmField var firstStack = 0.62
        @JvmField var stackPower = 1.0

        @JvmField var ejectPower = -0.5

        @JvmField var purplePower = -0.8
        @JvmField var purplePos = 0.4
    }

    private val servoRange = IntakeConfig.rangeStart..IntakeConfig.rangeEnd

    val angleServo = hardwareMap.rangedSpeedServo(
        deviceName =  "intakeAngle",
        direction = Servo.Direction.REVERSE,
        speed =  IntakeConfig.speed,
        range = servoRange
    )
    private val motor = hardwareMap.simpleMotor("intake", DcMotorSimple.Direction.REVERSE)

    private val time = DeltaTime()
    private var deltaTime: Time = 0.ms

    val pubDeltaTime by ::deltaTime

    var position by angleServo::position

    var targetPosition by angleServo::targetPosition

    var power by motor::power

    val isBusy by angleServo::isBusy

    fun stackPower() {
        power = IntakeConfig.stackPower
    }

    fun aboveStack() {
        position = IntakeConfig.aboveStackPose
    }

    fun firstStack() {
        position = IntakeConfig.firstStack
    }

    fun goDown() {
        position -= deltaTime.s
    }

    fun goUp() {
        position += deltaTime.s
    }

    fun update() {
        deltaTime = time.calculateDeltaTime()
        angleServo.update()
    }

    init {
        position = IntakeConfig.initPos
    }
}