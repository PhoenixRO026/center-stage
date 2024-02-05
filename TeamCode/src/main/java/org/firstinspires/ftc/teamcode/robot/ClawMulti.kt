package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.lib.units.SleepAction
import org.firstinspires.ftc.teamcode.lib.units.s
import java.util.concurrent.ConcurrentLinkedQueue

@Suppress("unused", "MemberVisibilityCanBePrivate")
class ClawMulti(
    hardwareMap: HardwareMap,
    private val innerClaw: Claw = Claw(hardwareMap)
) {
    val queue: ConcurrentLinkedQueue<() -> Unit> = ConcurrentLinkedQueue()

    companion object {
        fun clawMulti(hardwareMap: HardwareMap) = ClawMulti(hardwareMap)

        fun HardwareMap.clawMulti() = ClawMulti(this)
    }

    /*private var clawPositionCache = LazyWrite(
        writeT = { newValue -> innerClaw.clawPosition = newValue },
        getValueT = { innerClaw.clawPosition }
    )

    var clawPosition by clawPositionCache*/

    var clawPosition
        set(value) {
            queue.add {
                innerClaw.clawPosition = value
            }
        }
        get() = innerClaw.clawPosition

    /*private var clawTargetPositionCache = LazyWrite(
        writeT = { newValue -> innerClaw.clawTargetPosition = newValue },
        getValueT = { innerClaw.clawTargetPosition }
    )

    var clawTargetPosition by clawTargetPositionCache*/

    var clawTargetPosition by innerClaw::clawTargetPosition

    /*private var clawAngleCache = LazyWrite(
        writeT = { newValue -> innerClaw.clawAngle = newValue },
        getValueT = { innerClaw.clawAngle }
    )

    var clawAngle by clawAngleCache*/

    var clawAngle
        set(value) {
            queue.add {
                innerClaw.clawAngle = value
            }
        }
        get() = innerClaw.clawAngle

    /*private var clawTargetAngleCache = LazyWrite(
        writeT = { newValue -> innerClaw.clawTargetAngle = newValue },
        getValueT = { innerClaw.clawTargetAngle }
    )

    var clawTargetAngle by clawTargetAngleCache*/

    var clawTargetAngle by innerClaw::clawTargetAngle

    /*private var leftFingerPositionCache = LazyWrite(
        writeT = { newValue -> innerClaw.leftFingerPosition = newValue },
        getValueT = { innerClaw.leftFingerPosition }
    )

    var leftFingerPosition by leftFingerPositionCache*/

    var leftFingerPosition
        set(value) {
            queue.add {
                innerClaw.leftFingerPosition = value
            }
        }
        get() = innerClaw.leftFingerPosition

    /*private var leftFingerTargetPositionCache = LazyWrite(
        writeT = { newValue -> innerClaw.leftFingerTargetPosition = newValue },
        getValueT = { innerClaw.leftFingerTargetPosition }
    )

    var leftFingerTargetPosition by leftFingerTargetPositionCache*/

    private var leftFingerTargetPosition by innerClaw::leftFingerTargetPosition

    /*private var leftFingerAngleCache = LazyWrite(
        writeT = { newValue -> innerClaw.leftFingerAngle = newValue },
        getValueT = { innerClaw.leftFingerAngle }
    )

    var leftFingerAngle by leftFingerAngleCache*/

    var leftFingerAngle
        set(value) {
            queue.add {
                innerClaw.leftFingerAngle = value
            }
        }
        get() = innerClaw.leftFingerAngle

    /*private var leftFingerTargetAngleCache = LazyWrite(
        writeT = { newValue -> innerClaw.leftFingerTargetAngle = newValue },
        getValueT = { innerClaw.leftFingerTargetAngle }
    )

    var leftFingerTargetAngle by leftFingerTargetAngleCache*/

    private var leftFingerTargetAngle by innerClaw::leftFingerTargetAngle

    /*private var rightFingerPositionCache = LazyWrite(
        writeT = { newValue -> innerClaw.rightFingerPosition = newValue },
        getValueT = { innerClaw.rightFingerPosition }
    )

    var rightFingerPosition by rightFingerPositionCache*/

    var rightFingerPosition
        set(value) {
            queue.add {
                innerClaw.rightFingerPosition = value
            }
        }
        get() = innerClaw.rightFingerPosition

    /*private var rightFingerTargetPositionCache = LazyWrite(
        writeT = { newValue -> innerClaw.rightFingerTargetPosition = newValue },
        getValueT = { innerClaw.rightFingerTargetPosition }
    )

    var rightFingerTargetPosition by rightFingerTargetPositionCache*/

    private var rightFingerTargetPosition by innerClaw::rightFingerTargetPosition

    /*private var rightFingerAngleCache = LazyWrite(
        writeT = { newValue -> innerClaw.rightFingerAngle = newValue },
        getValueT = { innerClaw.rightFingerAngle }
    )

    var rightFingerAngle by rightFingerAngleCache*/

    var rightFingerAngle
        set(value) {
            queue.add {
                innerClaw.rightFingerAngle = value
            }
        }
        get() = innerClaw.rightFingerAngle

    /*private var rightFingerTargetAngleCache = LazyWrite(
        writeT = { newValue -> innerClaw.rightFingerTargetAngle = newValue },
        getValueT = { innerClaw.rightFingerTargetAngle }
    )

    var rightFingerTargetAngle by rightFingerTargetAngleCache*/

    private var rightFingerTargetAngle by innerClaw::rightFingerTargetAngle

    fun clawToPos(newPos: Double) = SequentialAction(
        InstantAction { clawPosition = newPos },
        SleepAction(0.1.s)
    )

    fun clawToRamp() = clawToPos(Claw.clawRampPos)

    fun leftFingerToPos(newPos: Double) = SequentialAction(
        InstantAction { leftFingerPosition = newPos },
        SleepAction(0.1.s)
    )

    fun openLeft() = leftFingerToPos(0.0)

    fun closeLeft() = leftFingerToPos(1.0)

    fun rightFingerToPos(newPos: Double) = SequentialAction(
        InstantAction { rightFingerPosition = newPos },
        SleepAction(0.1.s)
    )

    fun openRight() = rightFingerToPos(0.0)

    fun closeRight() = rightFingerToPos(1.0)

    fun closeClaw() = ParallelAction(
        openRight(),
        openLeft()
    )

    fun write() {
        while (queue.isNotEmpty()) {
            val action = queue.remove()
            action()
        }
    }
}