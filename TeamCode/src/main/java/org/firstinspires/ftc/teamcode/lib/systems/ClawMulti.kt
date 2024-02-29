package org.firstinspires.ftc.teamcode.lib.systems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.lib.units.SleepAction
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.s
import java.util.concurrent.ConcurrentLinkedQueue

@Suppress("unused", "MemberVisibilityCanBePrivate")
class ClawMulti(
    hardwareMap: HardwareMap,
    private val innerClaw: Claw = Claw(hardwareMap)
) {
    private val queue: ConcurrentLinkedQueue<() -> Unit> = ConcurrentLinkedQueue()

    companion object {
        fun clawMultiH(hardwareMap: HardwareMap) = ClawMulti(hardwareMap)

        fun HardwareMap.clawMulti() = ClawMulti(this)
    }

    var clawPosition by LazyWrite(
        queue,
        { innerClaw.clawPosition = it },
        { innerClaw.clawPosition }
    )

    var clawTargetPosition by innerClaw::clawTargetPosition

    var clawAngle by LazyWrite(
        queue,
        { innerClaw.clawAngle = it },
        { innerClaw.clawAngle }
    )

    var clawTargetAngle by innerClaw::clawTargetAngle

    var leftFingerPosition by LazyWrite(
        queue,
        { innerClaw.leftFingerPosition = it },
        { innerClaw.leftFingerPosition }
    )

    private var leftFingerTargetPosition by innerClaw::leftFingerTargetPosition

    var leftFingerAngle by LazyWrite(
        queue,
        { innerClaw.leftFingerAngle = it },
        { innerClaw.leftFingerAngle }
    )

    private var leftFingerTargetAngle by innerClaw::leftFingerTargetAngle

    var rightFingerPosition by LazyWrite(
        queue,
        { innerClaw.rightFingerPosition = it },
        { innerClaw.rightFingerPosition }
    )

    private var rightFingerTargetPosition by innerClaw::rightFingerTargetPosition

    var rightFingerAngle by LazyWrite(
        queue,
        { innerClaw.rightFingerAngle = it },
        { innerClaw.rightFingerAngle }
    )

    val isBusy by innerClaw::isBusy

    private var rightFingerTargetAngle by innerClaw::rightFingerTargetAngle

    fun clawToPos(newPos: Double) = SequentialAction(
        object : Action {
            var init = true

            override fun run(p: TelemetryPacket): Boolean {
                if (init) {
                    init = false
                    clawTargetPosition = newPos
                }

                return isBusy
            }
        },
        SleepAction(0.1.s)
    )

    fun clawToRamp() = clawToPos(Claw.rampPos)

    fun leftFingerToPos(newPos: Double) = SequentialAction(
        InstantAction { leftFingerPosition = newPos },
        SleepAction(0.2.s)
    )

    fun openLeft() = leftFingerToPos(0.0)

    fun closeLeft() = leftFingerToPos(1.0)

    fun openRamp() = ParallelAction(
        leftFingerToPos(Claw.fingerRampPos),
        rightFingerToPos(Claw.fingerRampPos)
    )

    fun clawToScore() = clawToPos(Claw.scorePos)

    fun rightFingerToPos(newPos: Double) = SequentialAction(
        InstantAction { rightFingerPosition = newPos },
        SleepAction(0.2.s)
    )

    fun openRight() = rightFingerToPos(0.0)

    fun closeRight() = rightFingerToPos(1.0)

    fun closeClaw() = ParallelAction(
        closeRight(),
        closeLeft()
    )

    fun update(deltaTime: Time) {
        innerClaw.update(deltaTime)
    }

    fun write() {
        while (queue.isNotEmpty()) {
            val action = queue.remove()
            action()
        }
    }
}