package org.firstinspires.ftc.teamcode.stc.robot

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.SequentialAction
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.lib.units.SleepAction
import org.firstinspires.ftc.teamcode.lib.units.s

class Plane(hardwareMap: HardwareMap) {
    private val tiltServo = hardwareMap.get(Servo::class.java, "tilt")
    private val launchServo = hardwareMap.get(Servo::class.java, "launch")

    companion object {
        const val holdTilt = 0.526
        const val launchTilt = 0.393
        const val holdPos = 0.76
        const val launch = 0.51
    }

    init {
        tiltServo.position = holdTilt
        launchServo.position = holdPos
    }

    private var launchAction: Action? = null

    var tiltPos: Double = holdTilt
        set(value) {
            tiltServo.position = value
            field = value.coerceIn(0.0, 1.0)
        }

    var launchPos: Double = holdPos
        set(value) {
            launchServo.position = value
            field = value.coerceIn(0.0, 1.0)
        }

    fun launch() {
        launchAction = SequentialAction(
            InstantAction { tiltPos = launchTilt },
            SleepAction(0.5.s),
            InstantAction { launchPos = launch },
            SleepAction(0.5.s),
            InstantAction { tiltPos = holdTilt }
        )
    }

    fun update() {
        val running = launchAction?.run(TelemetryPacket())
        if (running == false) launchAction = null
    }
}