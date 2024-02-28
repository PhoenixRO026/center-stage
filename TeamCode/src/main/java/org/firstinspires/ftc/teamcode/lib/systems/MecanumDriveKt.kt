package org.firstinspires.ftc.teamcode.lib.systems

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.HolonomicController
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.TimeTurn
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.ftc.throwIfModulesAreOutdated
import com.acmerobotics.roadrunner.now
import com.acmerobotics.roadrunner.range
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.lib.hardware.MotorEx.Companion.rev12to1
import org.firstinspires.ftc.teamcode.lib.localizer.ThreeWheelLocalizerEx
import org.firstinspires.ftc.teamcode.lib.units.rad
import org.firstinspires.ftc.teamcode.roadrunner.Drawing
import org.firstinspires.ftc.teamcode.roadrunner.Localizer
import java.util.LinkedList
import kotlin.math.max
import kotlin.math.roundToInt

@Suppress("MemberVisibilityCanBePrivate", "unused")
class MecanumDriveKt(
    hardwareMap: HardwareMap,
    var pose: Pose2d
) {
    @Config
    data object MecanumDriveKtParams {
        // IMU orientation
        // : fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        @JvmField var logoFacingDirection = LogoFacingDirection.UP
        @JvmField var usbFacingDirection = UsbFacingDirection.FORWARD

        // drive model parameters
        @JvmField var inPerTick = 1.0
        @JvmField var lateralInPerTick = inPerTick
        @JvmField var trackWidthTicks = 0.0

        // feedforward parameters (in tick units)
        @JvmField var kS = 0.0
        @JvmField var kV = 0.0
        @JvmField var kA = 0.0

        // path profile parameters (in inches)
        @JvmField var maxWheelVel = 50.0
        @JvmField var minProfileAccel = -30.0
        @JvmField var maxProfileAccel = 50.0

        // turn profile parameters (in radians)
        @JvmField var maxAngVel = Math.PI // shared with path

        @JvmField var maxAngAccel = Math.PI

        // path controller gains
        @JvmField var axialGain = 0.0
        @JvmField var lateralGain = 0.0
        @JvmField var headingGain = 0.0 // shared with turn

        @JvmField var axialVelGain = 0.0
        @JvmField var lateralVelGain = 0.0
        @JvmField var headingVelGain = 0.0 // shared with turn
    }

    val kinematics = MecanumKinematics(
        trackWidth = MecanumDriveKtParams.inPerTick * MecanumDriveKtParams.trackWidthTicks,
        lateralMultiplier = MecanumDriveKtParams.inPerTick / MecanumDriveKtParams.lateralInPerTick
    )

    val defaultTurnConstraints = TurnConstraints(
        maxAngVel = MecanumDriveKtParams.maxAngVel,
        minAngAccel = -MecanumDriveKtParams.maxAngAccel,
        maxAngAccel = MecanumDriveKtParams.maxAngAccel
    )

    val defaultVelConstraint = MinVelConstraint(listOf(
        kinematics.WheelVelConstraint(MecanumDriveKtParams.maxWheelVel),
        AngularVelConstraint(MecanumDriveKtParams.maxAngVel)
    ))

    val defaultAccelConstraint = ProfileAccelConstraint(
        minAccel = MecanumDriveKtParams.minProfileAccel,
        maxAccel = MecanumDriveKtParams.maxProfileAccel
    )

    val leftFront = hardwareMap.rev12to1("leftFront", DcMotorSimple.Direction.REVERSE)
    val leftBack = hardwareMap.rev12to1("leftBack", DcMotorSimple.Direction.REVERSE)
    val rightBack = hardwareMap.rev12to1("rightBack", DcMotorSimple.Direction.FORWARD)
    val rightFront = hardwareMap.rev12to1("rightFront", DcMotorSimple.Direction.FORWARD)

    val voltageSensor: VoltageSensor = hardwareMap.voltageSensor.iterator().next()

    val imu: IMU = hardwareMap.get(IMU::class.java, "imu")

    val localizer: Localizer = ThreeWheelLocalizerEx(
        hardwareMap,
        MecanumDriveKtParams.inPerTick
    )

    val poseHistory = LinkedList<Pose2d>()

    init {
        throwIfModulesAreOutdated(hardwareMap)

        imu.initialize(IMU.Parameters(RevHubOrientationOnRobot(
            MecanumDriveKtParams.logoFacingDirection,
            MecanumDriveKtParams.usbFacingDirection
        )))

        imu.resetYaw()
    }

    fun setDrivePowers(powers: PoseVelocity2d) {
        val wheelVels = MecanumKinematics(1.0)
            .inverse(PoseVelocity2dDual.constant<Time>(powers, 1))

        var maxPowerMag = 1.0

        for (power in wheelVels.all()) {
            maxPowerMag = max(maxPowerMag, power.value())
        }

        leftFront.power = wheelVels.leftFront.value() / maxPowerMag
        leftBack.power = wheelVels.leftBack.value() / maxPowerMag
        rightBack.power = wheelVels.rightBack.value() / maxPowerMag
        rightFront.power = wheelVels.rightFront.value() / maxPowerMag
    }

    inner class FollowTrajectoryAction(val timeTrajectory: TimeTrajectory) : Action {

        private var beginTs = -1.0
        private val xPoints: DoubleArray
        private val yPoints: DoubleArray

        init {
            val disps = range(
                begin = 0.0,
                end = timeTrajectory.path.length(),
                max(
                    2,
                    (timeTrajectory.path.length() / 2.0).roundToInt()
                )
            )

            xPoints = DoubleArray(disps.size)
            yPoints = DoubleArray(disps.size)

            for (i in disps.indices) {
                val p = timeTrajectory.path[disps[i], 1].value()
                xPoints[i] = p.position.x
                yPoints[i] = p.position.y
            }
        }

        override fun run(p: TelemetryPacket): Boolean {
            val t = if (beginTs < 0) {
                beginTs = now()
                0.0
            } else {
                now() - beginTs
            }

            if (t >= timeTrajectory.duration) {
                leftFront.power = 0.0
                leftBack.power = 0.0
                rightBack.power = 0.0
                rightFront.power = 0.0

                return false
            }

            val txWorldTarget = timeTrajectory[t]

            val robotVelRobot = updatePoseEstimate()

            /*val lastDisplacement = 0.0

            val pathDisplacement = project(timeTrajectory.path, pose.position, lastDisplacement)

            val targetPoseArc = timeTrajectory.path[pathDisplacement, 3]

            val timeDisplacement = timeTrajectory.profile.dispProfile[pathDisplacement]

            val targetPos = targetPoseArc.reparam(timeDisplacement)*/

            val command = HolonomicController(
                axialPosGain = MecanumDriveKtParams.axialGain,
                lateralPosGain = MecanumDriveKtParams.axialGain,
                headingGain = MecanumDriveKtParams.headingGain,
                axialVelGain = MecanumDriveKtParams.axialVelGain,
                lateralVelGain = MecanumDriveKtParams.lateralVelGain,
                headingVelGain = MecanumDriveKtParams.headingVelGain
            ).compute(
                targetPose = txWorldTarget,
                actualPose = pose,
                actualVelActual = robotVelRobot
            )

            val wheelVels = kinematics.inverse(command)
            val voltage = voltageSensor.voltage

            val feedforward = MotorFeedforward(
                kS = MecanumDriveKtParams.kS,
                kV = MecanumDriveKtParams.kV / MecanumDriveKtParams.inPerTick,
                kA = MecanumDriveKtParams.kA / MecanumDriveKtParams.inPerTick
            )

            leftFront.power = feedforward.compute(wheelVels.leftFront) / voltage
            leftBack.power = feedforward.compute(wheelVels.leftBack) / voltage
            rightBack.power = feedforward.compute(wheelVels.rightBack) / voltage
            rightFront.power = feedforward.compute(wheelVels.rightFront) / voltage

            p.put("x", pose.position.x)
            p.put("y", pose.position.y)
            p.put("heading (deg)", pose.heading.toDouble().rad.deg)

            val error = txWorldTarget.value().minusExp(pose)

            p.put("xError", error.position.x)
            p.put("yError", error.position.y)
            p.put("headingError (deg)", error.heading.toDouble().rad.deg)

            // only draw when active; only one drive action should be active at a time
            val c = p.fieldOverlay()
            drawPoseHistory(c)

            c.setStroke("#4CAF50")
            Drawing.drawRobot(c, txWorldTarget.value())

            c.setStroke("#3F51B5")
            Drawing.drawRobot(c, pose)

            c.setStroke("#4CAF50FF")
            c.setStrokeWidth(1)
            c.strokePolyline(xPoints, yPoints)

            return true
        }

        override fun preview(fieldOverlay: Canvas) {
            fieldOverlay.setStroke("#4CAF507A")
            fieldOverlay.setStrokeWidth(1)
            fieldOverlay.strokePolyline(xPoints, yPoints)
        }

    }

    inner class TurnAction(val turn: TimeTurn) : Action {

        private var beginTs = -1.0

        override fun run(p: TelemetryPacket): Boolean {
            val t = if (beginTs < 0) {
                beginTs = now()
                0.0
            } else {
                now() - beginTs
            }

            if (t >= turn.duration) {
                leftFront.power = 0.0
                leftBack.power = 0.0
                rightBack.power = 0.0
                rightFront.power = 0.0

                return false
            }

            val txWorldTarget = turn[t]

            val robotVelRobot = updatePoseEstimate()

            val command = HolonomicController(
                axialPosGain = MecanumDriveKtParams.axialGain,
                lateralPosGain = MecanumDriveKtParams.axialGain,
                headingGain = MecanumDriveKtParams.headingGain,
                axialVelGain = MecanumDriveKtParams.axialVelGain,
                lateralVelGain = MecanumDriveKtParams.lateralVelGain,
                headingVelGain = MecanumDriveKtParams.headingVelGain
            ).compute(
                targetPose = txWorldTarget,
                actualPose = pose,
                actualVelActual = robotVelRobot
            )

            val wheelVels = kinematics.inverse(command)
            val voltage = voltageSensor.voltage

            val feedforward = MotorFeedforward(
                kS = MecanumDriveKtParams.kS,
                kV = MecanumDriveKtParams.kV / MecanumDriveKtParams.inPerTick,
                kA = MecanumDriveKtParams.kA / MecanumDriveKtParams.inPerTick
            )

            leftFront.power = feedforward.compute(wheelVels.leftFront) / voltage
            leftBack.power = feedforward.compute(wheelVels.leftBack) / voltage
            rightBack.power = feedforward.compute(wheelVels.rightBack) / voltage
            rightFront.power = feedforward.compute(wheelVels.rightFront) / voltage

            val c = p.fieldOverlay()
            drawPoseHistory(c)

            c.setStroke("#4CAF50")
            Drawing.drawRobot(c, txWorldTarget.value())

            c.setStroke("#3F51B5")
            Drawing.drawRobot(c, pose)

            c.setStroke("#7C4DFFFF")
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2.0)


            return true
        }

        override fun preview(fieldOverlay: Canvas) {
            fieldOverlay.setStroke("#7C4DFF7A")
            fieldOverlay.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2.0)
        }

    }

    fun updatePoseEstimate(): PoseVelocity2d {
        val twist = localizer.update()
        pose += twist.value()

        poseHistory.add(pose)
        while (poseHistory.size > 100) {
            poseHistory.removeFirst()
        }

        return twist.velocity().value()
    }

    private fun drawPoseHistory(c: Canvas) {
        val xPoints = DoubleArray(poseHistory.size)
        val yPoints = DoubleArray(poseHistory.size)

        for (i in poseHistory.indices) {
            xPoints[i] = poseHistory[i].position.x
            yPoints[i] = poseHistory[i].position.y
        }

        c.setStrokeWidth(1)
        c.setStroke("#3F51B5")
        c.strokePolyline(xPoints, yPoints)
    }

    fun actionBuilder(beginPose: Pose2d): TrajectoryActionBuilder {
        return TrajectoryActionBuilder(
            turnActionFactory = { TurnAction(it) },
            trajectoryActionFactory = { FollowTrajectoryAction(it) },
            beginPose = beginPose,
            eps = 1e-6,
            beginEndVel = 0.0,
            baseTurnConstraints = defaultTurnConstraints,
            baseVelConstraint = defaultVelConstraint,
            baseAccelConstraint = defaultAccelConstraint,
            dispResolution = 0.25,
            angResolution = 0.1
        )
    }
}