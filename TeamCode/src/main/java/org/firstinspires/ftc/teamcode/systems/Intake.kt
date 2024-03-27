package org.firstinspires.ftc.teamcode.systems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.lib.hardware.motor.SimpleMotor.Companion.simpleMotor
import org.firstinspires.ftc.teamcode.lib.hardware.servo.RangedSpeedServo.Companion.rangedSpeedServo
import com.phoenix.phoenixlib.units.DeltaTime
import com.phoenix.phoenixlib.units.Time
import com.phoenix.phoenixlib.units.ms

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
        @JvmField var aboveFirstStack = 0.628
        @JvmField var firstStack = 0.614
        @JvmField var aboveSecondStack = 0.43
        @JvmField var secondStack = 0.35
        @JvmField var aboveThirdStack = 0.1
        @JvmField var thirdStack = 0.0
        @JvmField var forthStack = 0.6
        @JvmField var stackPower = 1.0

        @JvmField var boardSideAboveFirstStack = 0.628
        @JvmField var boardSideFirstStack = 0.614
        @JvmField var boardSideAboveSecondStack = 0.43
        @JvmField var boardSideSecondStack = 0.35

        @JvmField var ejectPower = -1.0

        @JvmField var purplePower = -0.8
        @JvmField var purplePos = 0.4

        @JvmField var onePixelStackWaitSec = 0.5
        @JvmField var twoPixelStackWaitSec = 0.5

        @JvmField var ejectPixelTimeSec = 2.0
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
        position = IntakeConfig.aboveFirstStack
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