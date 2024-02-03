package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.SequentialAction
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.lib.units.Angle
import org.firstinspires.ftc.teamcode.lib.units.SleepAction
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.deg
import org.firstinspires.ftc.teamcode.lib.units.s

@Suppress("unused", "MemberVisibilityCanBePrivate")
class Arm(
    hardwareMap: HardwareMap
) {
    companion object {
        const val servoOffset = 0.015
        val leftServoRange = servoOffset..1.0
        val rightServoRange = 0.0..(1.0 - servoOffset)
        const val speed = 0.1
        const val rampPos = 0.395

        fun HardwareMap.arm() = Arm(this)
    }

    private val rightServo = ServoEx(
        hardwareMap.get(Servo::class.java, "rightArm"),
        355.deg,
        rightServoRange,
        speed = speed,
        onPositionUpdate = ::updateLeftServo
    )
    private val leftServo = ServoEx(
        hardwareMap.get(Servo::class.java, "leftArm"),
        355.deg,
        leftServoRange,
        changeTreshold = 0.0
    )

    private fun updateLeftServo() {
        leftServo.position = rightServo.position
    }

    var position by rightServo::position

    var targetPosition by rightServo::position

    var angle by rightServo::angle

    var targetAngle by rightServo::angle

    val isBusy by rightServo::isBusy

    fun goToPos(newPos: Double) = SequentialAction(
        object : Action {
            var init = true

            override fun run(p: TelemetryPacket): Boolean {
                if (init) {
                    init = false
                    targetPosition = newPos
                }

                return isBusy
            }
        },
        SleepAction(0.1.s)
    )

    fun goToAngle(newAngle: Angle) = SequentialAction(
        object : Action {
            var init = true

            override fun run(p: TelemetryPacket): Boolean {
                if (init) {
                    init = false
                    targetAngle = newAngle
                }

                return isBusy
            }
        },
        SleepAction(0.1.s)
    )

    fun update(deltaTime: Time) {
        rightServo.update(deltaTime)
    }

    init {
        position = rampPos
    }
}