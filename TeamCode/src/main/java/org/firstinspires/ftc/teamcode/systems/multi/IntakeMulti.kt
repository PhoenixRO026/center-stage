package org.firstinspires.ftc.teamcode.systems.multi

import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.SequentialAction
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.lib.multi.LazyWrite
import org.firstinspires.ftc.teamcode.lib.units.SleepAction
import org.firstinspires.ftc.teamcode.lib.units.s
import org.firstinspires.ftc.teamcode.systems.Arm.Companion.arm
import org.firstinspires.ftc.teamcode.systems.Intake
import org.firstinspires.ftc.teamcode.systems.Intake.Companion.intake
import java.util.concurrent.ConcurrentLinkedQueue

class IntakeMulti(
        private val innerIntake: Intake
) {
    companion object {
        fun HardwareMap.intakeMulti() = IntakeMulti(this.intake())
    }

    private val queue = ConcurrentLinkedQueue<LazyWrite.Task>()

    var position by LazyWrite(
            queue,
            { innerIntake.position = it },
            { innerIntake.position }
    )

    var targetPosition by innerIntake::targetPosition

    var power by LazyWrite(
            queue,
            { innerIntake.power = it },
            { innerIntake.power }
    )

    val isBusy by innerIntake::isBusy

    fun ejectPixels() = SequentialAction(
        InstantAction { power = Intake.IntakeConfig.ejectPower },
        SleepAction(1.s),
        InstantAction { power = 0.0 }
    )

    fun stackPower() {
        power = Intake.IntakeConfig.stackPower
    }

    fun ejectPurple() = SequentialAction(
        InstantAction { position = Intake.IntakeConfig.purplePos },
        SleepAction(0.08.s),
        InstantAction { power = Intake.IntakeConfig.purplePower },
        SleepAction(0.3.s)
    )

    fun aboveStack() {
        position = Intake.IntakeConfig.aboveStackPose
    }

    fun firstStack() {
        targetPosition = Intake.IntakeConfig.firstStack
    }

    fun goDown() {
        position -= innerIntake.pubDeltaTime.s
    }

    fun goUp() {
        position += innerIntake.pubDeltaTime.s
    }

    fun write() {
        var task: LazyWrite.Task? = queue.poll()

        while (task != null) {
            task.task()
            task = queue.poll()
        }
    }

    fun update() {
        innerIntake.update()
    }
}