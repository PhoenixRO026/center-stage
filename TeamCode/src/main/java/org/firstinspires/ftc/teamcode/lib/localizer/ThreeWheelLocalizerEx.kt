package org.firstinspires.ftc.teamcode.lib.localizer

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.Twist2dDual
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.Vector2dDual
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.roadrunner.Localizer
import org.mercurialftc.mercurialftc.silversurfer.geometry.matrix.SimpleMatrix
import kotlin.math.cos
import kotlin.math.sin

class ThreeWheelLocalizerEx(
    hardwareMap: HardwareMap,
    private val inPerTick: Double
) : Localizer{
    @Config
    data object ThreeWheelParams {
        @JvmField var trackWidthTicks: Double = 5221.878178997774 + 4557.065579735478
        @JvmField var perpXTicks: Double = -5567.90748691027
    }

    private val par0 = OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, "leftBack")))
    private val par1 = OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, "rightBack")))
    private val perp = OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, "leftFront")))

    private var initialized = false
    private var lastPar0Pos = 0.0
    private var lastPar1Pos = 0.0
    private var lastPerpPos = 0.0

    init {
        par0.direction = DcMotorSimple.Direction.REVERSE
        par1.direction = DcMotorSimple.Direction.REVERSE
        perp.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun update(): Twist2dDual<Time> {
        val par0PosVel = par0.getPositionAndVelocity()
        val par1PosVel = par1.getPositionAndVelocity()
        val perpPosVel = perp.getPositionAndVelocity()

        val par0Pos = par0PosVel.position.toDouble()
        val par1Pos = par1PosVel.position.toDouble()
        val perpPos = perpPosVel.position.toDouble()

        val par0Vel = par0PosVel.velocity.toDouble()
        val par1Vel = par1PosVel.velocity.toDouble()
        val perpVel = perpPosVel.velocity.toDouble()

        if (!initialized) {
            initialized = true

            lastPar0Pos = par0Pos
            lastPar1Pos = par1Pos
            lastPerpPos = perpPos

            return Twist2dDual(
                Vector2dDual.constant(Vector2d(0.0, 0.0), 2),
                DualNum.constant(0.0, 2)
            )
        }

        val par0PosDelta = par0Pos - lastPar0Pos
        val par1PosDelta = par1Pos - lastPar1Pos
        val perpPosDelta = perpPos - lastPerpPos

        lastPar0Pos = par0Pos
        lastPar1Pos = par1Pos
        lastPerpPos = perpPos

        val deltaTheta = (par0PosDelta - par1PosDelta) / ThreeWheelParams.trackWidthTicks
        val deltaX = perpPosDelta - ThreeWheelParams.perpXTicks * deltaTheta
        val deltaY = (par0PosDelta + par1PosDelta) / 2.0

        val thetaVel = (par0Vel - par1Vel) / ThreeWheelParams.trackWidthTicks
        val xVel = perpVel - ThreeWheelParams.perpXTicks * thetaVel
        val yVel = (par0Vel + par1Vel) / 2.0

        val (term0, term1) = if (deltaTheta == 0.0)
            Pair(1.0, 0.0)
        else
            Pair(
                sin(deltaTheta) / deltaTheta,
                (1.0 - cos(deltaTheta)) / deltaTheta
            )

        val twistMatrix = SimpleMatrix(
            arrayOf(
                doubleArrayOf(term0, -term1, 0.0),
                doubleArrayOf(term1, term0, 0.0),
                doubleArrayOf(0.0, 0.0, 1.0)
            )
        )
        val inputMatrix = SimpleMatrix(
            arrayOf(
                doubleArrayOf(deltaX),
                doubleArrayOf(deltaY),
                doubleArrayOf(deltaTheta)
            )
        )

        val twistResult = twistMatrix.multiply(inputMatrix)

        val twistDeltaX = twistResult.getItem(0, 0)
        val twistDeltaY = twistResult.getItem(1, 0)
        val twistDeltaTheta = twistResult.getItem(2, 0)

        return Twist2dDual(
            Vector2dDual(
                DualNum<Time>(listOf(twistDeltaY, yVel)).times(inPerTick),
                DualNum<Time>(listOf(twistDeltaX, xVel)).times(inPerTick)
            ),
            DualNum(listOf(twistDeltaTheta, thetaVel))
        )
    }
}