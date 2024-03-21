package org.firstinspires.ftc.teamcode.systems.multi

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.SequentialAction
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.lib.multi.LazyWrite
import org.firstinspires.ftc.teamcode.lib.units.SleepAction
import org.firstinspires.ftc.teamcode.lib.units.s
import org.firstinspires.ftc.teamcode.systems.Arm
import org.firstinspires.ftc.teamcode.systems.Arm.Companion.arm
import java.util.concurrent.ConcurrentLinkedQueue

class ArmMulti(
        private val innerArm: Arm
) {
    companion object {
        fun HardwareMap.armMulti() = ArmMulti(this.arm())
    }

    private val queue = ConcurrentLinkedQueue<LazyWrite.Task>()

    var position by LazyWrite(
            queue,
            { innerArm.position = it },
            { innerArm.position }
    )

    var targetPosition by innerArm::targetPosition

    val isBusy by innerArm::isBusy

    fun intakePos() {
        targetPosition = Arm.ArmConfig.intakePos
    }

    fun scorePos() {
        targetPosition = Arm.ArmConfig.scorePos
    }

    fun intakePosQuick() = SequentialAction(
        InstantAction { position = Arm.ArmConfig.intakePos },
        Action { position != Arm.ArmConfig.intakePos },
        SleepAction(Arm.ArmConfig.intakeToScoreTravelWaitSec.s)
    )

    fun scorePosQuick() = SequentialAction(
        InstantAction { position = Arm.ArmConfig.scorePos },
        Action { position != Arm.ArmConfig.scorePos },
        SleepAction(Arm.ArmConfig.intakeToScoreTravelWaitSec.s)
    )

    fun intakePosNow() {
        position = Arm.ArmConfig.intakePos
    }

    fun scorePosNow() {
        position = Arm.ArmConfig.scorePos
    }

    fun goToIntake() = object : Action {
        var init = true
        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false
                innerArm.intakePos()
            }

            return isBusy
        }
    }

    fun goToScore() = object : Action {
        var init = true
        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false
                innerArm.scorePos()
            }

            return isBusy
        }
    }

    fun write() {
        var task: LazyWrite.Task? = queue.poll()

        while (task != null) {
            task.task()
            task = queue.poll()
        }
    }

    fun update() {
        innerArm.update()
    }
}