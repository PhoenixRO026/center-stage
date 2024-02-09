package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.robot.hardware.MotorEx.Companion.gobilda312

@Suppress("unused", "MemberVisibilityCanBePrivate")
class Lift(
    hardwareMap: HardwareMap
) {
    companion object {
        const val hangTicks = 600
        const val rampTicks = 70
        const val autoRampTicks = 200
        const val passTicks = 1500
        const val aboveYellowTicks = 1400
        const val yellowPixelTicks = 1250
        const val toleranceTicks = 10

        fun liftH(hardwareMap: HardwareMap) = Lift(hardwareMap)

        fun HardwareMap.lift() = Lift(this)
    }

    enum class MODE {
        RAW,
        HANG,
        RAMP,
        TARGET_POS
    }

    private val leftMotor = hardwareMap.gobilda312(
        deviceName = "leftLift",
        //onPowerUpdate = ::updateRightMotor,
        changeThreshold = 0.0
    )
    private val rightMotor = hardwareMap.gobilda312(
        deviceName = "rightLift",
        direction = DcMotorSimple.Direction.REVERSE,
        changeThreshold = 0.0
    )

    /*private fun updateRightMotor() {
        rightMotor.power = leftMotor.power
    }*/

    private var mode = MODE.RAW
        set(value) {
            if (field != value) {
                when (value) {
                    MODE.RAW -> {
                        leftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                        rightMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                        power = 0.0
                    }
                    MODE.HANG -> {
                        leftMotor.targetPositionTicks = hangTicks
                        rightMotor.targetPositionTicks = hangTicks
                        leftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                        rightMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                        leftMotor.power = 0.5
                        rightMotor.power = 0.5
                    }
                    MODE.RAMP -> {
                        leftMotor.targetPositionTicks = rampTicks
                        rightMotor.targetPositionTicks = rampTicks
                        leftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                        rightMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                        leftMotor.power = 1.0
                        rightMotor.power = 1.0
                    }
                    MODE.TARGET_POS -> {
                        leftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                        rightMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                        leftMotor.power = 1.0
                        rightMotor.power = 1.0
                    }
                }
            }
            field = value
        }

    var power
        get() = leftMotor.power
        set(value) {
            if (mode == MODE.RAMP && value != 0.0) {
                mode = MODE.RAW
            }
            if (mode == MODE.RAW) {
                leftMotor.power = value
                rightMotor.power = value
            }
        }

    val position by leftMotor::positionTicks

    val angle by leftMotor::angle

    val isBusy get() = leftMotor.isBusy || rightMotor.isBusy

    fun goToTicksAsync(ticks: Int) {
        leftMotor.targetPositionTicks = ticks
        rightMotor.targetPositionTicks = ticks
        mode = MODE.TARGET_POS
    }

    fun goToTicks(ticks: Int) = object : Action {
        var init = true

        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false
                goToTicksAsync(ticks)
            }

            return isBusy
        }
    }

    fun goToRampAsync() {
        mode = MODE.RAMP
    }

    fun hangAsync() {
        mode = MODE.HANG
    }

    fun unhangAsync() {
        mode = MODE.RAW
    }
}