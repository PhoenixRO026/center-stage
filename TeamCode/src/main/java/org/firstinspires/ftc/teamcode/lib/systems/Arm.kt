package org.firstinspires.ftc.teamcode.lib.systems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.SequentialAction
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.lib.hardware.ServoEx
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
        const val speed = 0.6
        const val rampPos = 0.427
        const val scorePos = 0.756

        fun HardwareMap.arm() = Arm(this)
    }

    private val rightServo = ServoEx(
        servo = hardwareMap.get(Servo::class.java, "rightArm"),
        maxAngle =  355.deg,
        positionRange = rightServoRange,
        speed = speed,
        changeTreshold = 0.0
    )
    private val leftServo = ServoEx(
        servo =  hardwareMap.get(Servo::class.java, "leftArm"),
        maxAngle = 355.deg,
        positionRange = leftServoRange,
        changeTreshold = 0.0
    )

    val realPosition by rightServo::currentUnscaledPosition

    var position
        get() = rightServo.position
        set(value) {
            rightServo.position = value
            leftServo.position = value
        }

    var targetPosition by rightServo::targetPosition

    var angle by rightServo::angle

    var targetAngle by rightServo::targetAngle

    val isBusy by rightServo::isBusy

    fun goToRamp() = goToPos(rampPos)

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
        leftServo.position = rightServo.position
    }

    init {
        //position = rampPos /*+ 0.02*/
        rightServo.position = rampPos
        leftServo.position = rampPos
    }
}