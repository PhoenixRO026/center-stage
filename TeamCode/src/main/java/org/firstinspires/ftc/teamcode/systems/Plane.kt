package org.firstinspires.ftc.teamcode.systems

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.SequentialAction
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.lib.units.SleepAction
import org.firstinspires.ftc.teamcode.lib.units.s

class Plane(
        hardwareMap: HardwareMap
) {
    companion object {
        fun HardwareMap.plane() = Plane(this)
    }

    @Config
    data object PlaneConfig {
        @JvmField var tiltRest = 0.4044
        @JvmField var tiltLaunch = 0.568
        @JvmField var launchPlane = 0.1
        @JvmField var hold = 0.15333
    }

    private var launchAction: Action? = null

//    private val tilt = hardwareMap.simpleServo("planeTilt")
//    private val launch = hardwareMap.simpleServo("plane")

    private val tilt = hardwareMap.get(Servo::class.java, "planeTilt")
    private val launch = hardwareMap.get(Servo::class.java, "plane")

    var tiltPosition by tilt::position

    var launchPosition by launch::position

    fun launch() {
        launchAction = SequentialAction(
                InstantAction { tiltPosition = PlaneConfig.tiltLaunch },
                SleepAction(0.5.s),
                InstantAction { launchPosition = PlaneConfig.launchPlane },
                SleepAction(1.s),
                InstantAction {
                    tiltPosition = PlaneConfig.tiltRest
                    launchPosition = PlaneConfig.hold
                }
        )
    }

    fun update() {
        launchAction?.let {
            if (!it.run(TelemetryPacket())) {
                launchAction = null
            }
        }
    }

    init {
        tiltPosition = PlaneConfig.tiltRest
        launchPosition = PlaneConfig.hold
    }
}