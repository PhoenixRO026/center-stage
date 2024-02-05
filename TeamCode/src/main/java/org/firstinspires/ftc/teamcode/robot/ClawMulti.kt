package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import org.firstinspires.ftc.teamcode.lib.units.SleepAction
import org.firstinspires.ftc.teamcode.lib.units.s

@Suppress("unused", "MemberVisibilityCanBePrivate")
class ClawMulti(
    private val innerClaw: Claw
) {
    companion object {

    }

    private var clawPositionCache = LazyWrite(
        writeT = { newValue -> innerClaw.clawPosition = newValue },
        getValueT = { innerClaw.clawPosition }
    )

    var clawPosition by clawPositionCache

    private var clawTargetPositionCache = LazyWrite(
        writeT = { newValue -> innerClaw.clawTargetPosition = newValue },
        getValueT = { innerClaw.clawTargetPosition }
    )

    var clawTargetPosition by clawTargetPositionCache

    private var clawAngleCache = LazyWrite(
        writeT = { newValue -> innerClaw.clawAngle = newValue },
        getValueT = { innerClaw.clawAngle }
    )

    var clawAngle by clawAngleCache

    private var clawTargetAngleCache = LazyWrite(
        writeT = { newValue -> innerClaw.clawTargetAngle = newValue },
        getValueT = { innerClaw.clawTargetAngle }
    )

    var clawTargetAngle by clawTargetAngleCache

    private var leftFingerPositionCache = LazyWrite(
        writeT = { newValue -> innerClaw.leftFingerPosition = newValue },
        getValueT = { innerClaw.leftFingerPosition }
    )

    var leftFingerPosition by leftFingerPositionCache

    private var leftFingerTargetPositionCache = LazyWrite(
        writeT = { newValue -> innerClaw.leftFingerTargetPosition = newValue },
        getValueT = { innerClaw.leftFingerTargetPosition }
    )

    var leftFingerTargetPosition by leftFingerTargetPositionCache

    private var leftFingerAngleCache = LazyWrite(
        writeT = { newValue -> innerClaw.leftFingerAngle = newValue },
        getValueT = { innerClaw.leftFingerAngle }
    )

    var leftFingerAngle by leftFingerAngleCache

    private var leftFingerTargetAngleCache = LazyWrite(
        writeT = { newValue -> innerClaw.leftFingerTargetAngle = newValue },
        getValueT = { innerClaw.leftFingerTargetAngle }
    )

    var leftFingerTargetAngle by leftFingerTargetAngleCache

    private var rightFingerPositionCache = LazyWrite(
        writeT = { newValue -> innerClaw.rightFingerPosition = newValue },
        getValueT = { innerClaw.rightFingerPosition }
    )

    var rightFingerPosition by rightFingerPositionCache

    private var rightFingerTargetPositionCache = LazyWrite(
        writeT = { newValue -> innerClaw.rightFingerTargetPosition = newValue },
        getValueT = { innerClaw.rightFingerTargetPosition }
    )

    var rightFingerTargetPosition by rightFingerTargetPositionCache

    private var rightFingerAngleCache = LazyWrite(
        writeT = { newValue -> innerClaw.rightFingerAngle = newValue },
        getValueT = { innerClaw.rightFingerAngle }
    )

    var rightFingerAngle by rightFingerAngleCache

    private var rightFingerTargetAngleCache = LazyWrite(
        writeT = { newValue -> innerClaw.rightFingerTargetAngle = newValue },
        getValueT = { innerClaw.rightFingerTargetAngle }
    )

    var rightFingerTargetAngle by rightFingerTargetAngleCache

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
        clawPositionCache.write()
        clawTargetPositionCache.write()
        clawAngleCache.write()
        clawTargetAngleCache.write()
        leftFingerPositionCache.write()
        leftFingerTargetPositionCache.write()
        leftFingerAngleCache.write()
        leftFingerTargetAngleCache.write()
        rightFingerPositionCache.write()
        rightFingerTargetPositionCache.write()
        rightFingerAngleCache.write()
        rightFingerTargetAngleCache.write()
    }
}