package org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.systems

import com.arcrobotics.ftclib.command.ProfiledPIDCommand
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.CONFIG
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.hardware.MotorEx

class Lift(
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

    val mode = MODE.RAW

    var power: Double = 0.0
        set(value) {
            leftMotor.power = value
            rightMotor.power = value
            field = value
        }

    val leftPosition: Int
        get() = leftMotor.position

    var goal: Double = 0.0
        set(value) {
            controller.setGoal(goal)
            field = value
        }

    fun update() {
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