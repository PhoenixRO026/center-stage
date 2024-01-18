package org.firstinspires.ftc.teamcode.evenimente.beclean.robot.systems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.HANG_POS
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.CONFIG
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.hardware.MotorEx
import kotlin.math.abs

class Lift2(
    hardwareMap: HardwareMap,
    private val telemetry: Telemetry? = null
) {
    enum class MODE {
        RAW,
        MOTION_PROFILED,
        HANG
    }

    data object LiftConfig {
        @JvmField var P = 0.0
        @JvmField var I = 0.0
        @JvmField var D = 0.0
        @JvmField var maxVel = 0.0
        @JvmField var maxAccel = 0.0
        private var lastP = P
        private var lastI = I
        private var lastD = D
        private var lastMaxVel = maxVel
        private var lastMaxAccel = maxAccel
        val changed: Boolean
            get() = P != lastP ||
                    I != lastI ||
                    D != lastD ||
                    maxVel != lastMaxVel ||
                    maxAccel != lastMaxAccel

        fun update() {
            lastP = P
            lastI = I
            lastD = D
            lastMaxVel = maxVel
            lastMaxAccel = maxAccel
        }
    }

    private val leftMotor = MotorEx(hardwareMap, CONFIG.LEFT_LIFT, telemetry, DcMotorSimple.Direction.REVERSE)
    private val rightMotor = MotorEx(hardwareMap, CONFIG.RIGHT_LIFT, telemetry)

    private val controller = ProfiledPIDController(
        LiftConfig.P,
        LiftConfig.I,
        LiftConfig.D,
        TrapezoidProfile.Constraints(
            LiftConfig.maxVel,
            LiftConfig.maxAccel
        )
    )

    var mode = MODE.RAW

    var power: Double = 0.0
        set(value) {
            if (mode == MODE.RAW) {
                leftMotor.power = value
                rightMotor.power = value
            }
            field = value
        }

    var breakingMode = ZeroPowerBehavior.BRAKE
        set(value) {
            leftMotor.zeroPowerBehavior = value
            rightMotor.zeroPowerBehavior = value
            field = value
        }

    val leftPosition: Int
        get() = leftMotor.position

    val busy = leftMotor.busy || rightMotor.busy

    var goal: Double = 0.0
        set(value) {
            controller.setGoal(goal)
            field = value
        }

    fun goToPos(pos: Int): Action = object : Action {
        var init = true

        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false
                mode = MODE.RAW
                leftMotor.targetPosition = pos
                rightMotor.targetPosition = pos
                leftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                rightMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                power = 1.0
            }

            return busy || abs(leftMotor.targetPosition - leftPosition) > 5
        }
    }

    fun hang() {
        if (mode == MODE.HANG) return
        mode = MODE.HANG
        leftMotor.targetPosition = HANG_POS
        rightMotor.targetPosition = HANG_POS
        leftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        rightMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        leftMotor.power = 0.5
        rightMotor.power = 0.5
        power = 0.5
    }

    fun unhang() {
        if (mode == MODE.RAW) return
        mode = MODE.RAW
        leftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun update() {
        telemetry?.addData("busy", busy)
        if (LiftConfig.changed) {
            controller.setPID(LiftConfig.P, LiftConfig.I, LiftConfig.D)
            controller.setConstraints(TrapezoidProfile.Constraints(
                LiftConfig.maxVel,
                LiftConfig.maxAccel
            ))
            LiftConfig.update()
        }
        if (mode == MODE.MOTION_PROFILED) {
            leftMotor.power = controller.calculate(leftMotor.position.toDouble())
            rightMotor.power = controller.calculate(rightMotor.position.toDouble())
        }
    }
}