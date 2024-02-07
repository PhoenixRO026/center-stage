package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.robot.hardware.MotorEx.Companion.gobilda435
import org.firstinspires.ftc.teamcode.robot.hardware.ServoEx.Companion.axonMax180

@Suppress("unused", "MemberVisibilityCanBePrivate")
class Intake(
    hardwareMap: HardwareMap
) {
    @Config
    data object IntakeConfig {
        @JvmField
        var stack1 = 0.62
        var stack2 = 0.64
    }
    companion object {
        val intakeRange = 0.489..0.77

        const val aboveStack = 0.5

        const val speed = 0.1

        fun intakeH(hardwareMap: HardwareMap) = Intake(hardwareMap)

        fun HardwareMap.intake() = Intake(this)
    }

    private val motor = hardwareMap.gobilda435(
        deviceName = "intake",
        direction = DcMotorSimple.Direction.REVERSE
    )
    private val servo = hardwareMap.axonMax180(
        deviceName = "intakeServo",
        range = intakeRange,
        changeTreshold = 0.0,
        speed = Intake.speed
    )

    var power by motor::power

    var position by servo::position

    var targetPosition by servo::targetPosition

    var speed by servo::speed

    val isBusy by servo::isBusy

    fun update(deltaTime: Time) {
        servo.update(deltaTime)
    }

    init {
        position = 0.0
    }
}