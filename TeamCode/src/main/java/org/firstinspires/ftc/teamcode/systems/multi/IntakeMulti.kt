package org.firstinspires.ftc.teamcode.systems.multi

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.SequentialAction
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.lib.multi.LazyWrite
import org.firstinspires.ftc.teamcode.lib.units.SleepAction
import org.firstinspires.ftc.teamcode.lib.units.s
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
        SleepAction(Intake.IntakeConfig.ejectPixelTimeSec.s),
        InstantAction { power = 0.0 }
    )

    fun stackPower() {
        power = Intake.IntakeConfig.stackPower
    }

    fun waitForPixel() = SleepAction(Intake.IntakeConfig.onePixelStackWaitSec.s)

    fun ejectPurple() = SequentialAction(
        InstantAction { position = Intake.IntakeConfig.purplePos },
        Action { position != Intake.IntakeConfig.purplePos },
        SleepAction(0.13.s),
        InstantAction { power = Intake.IntakeConfig.purplePower },
        SleepAction(0.3.s)
    )

    fun aboveFirstStack() {
        position = Intake.IntakeConfig.aboveFirstStack
    }

    fun firstStack() {
        targetPosition = Intake.IntakeConfig.firstStack
    }

    fun aboveSecondStack() {
        position = Intake.IntakeConfig.aboveSecondStack
    }

    fun secondStack() {
        targetPosition = Intake.IntakeConfig.secondStack
    }

    fun aboveThirdStack() {
        position = Intake.IntakeConfig.aboveThirdStack
    }

    fun thirdStack() {
        targetPosition = Intake.IntakeConfig.thirdStack
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