package org.firstinspires.ftc.teamcode.evenimente.liga_phoenix.robot

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.HolonomicController
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.TimeTurn
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.Twist2dDual
import com.acmerobotics.roadrunner.VelConstraint
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.acmerobotics.roadrunner.ftc.throwIfModulesAreOutdated
import com.acmerobotics.roadrunner.now
import com.acmerobotics.roadrunner.range
import com.phoenix_ro026.phoenixlib.units.Pose
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.evenimente.liga_phoenix.get
import org.firstinspires.ftc.teamcode.roadrunner.Localizer
import java.util.LinkedList
import kotlin.math.ceil

class MecanumDrive(hardwareMap: HardwareMap, initPose2d: Pose2d) {
    constructor(hardwareMap: HardwareMap, initPose: Pose) : this(hardwareMap, initPose.toPose2d())

    data object Params {
        // IMU orientation
        val logoFacingDirection = LogoFacingDirection.FORWARD
        val usbFacingDirection = UsbFacingDirection.LEFT

        // drive model parameters
        const val inPerTick = 1.0
        const val lateralInPerTick = 1.0
        const val trackWidthTicks = 0.0

        // feedforward parameters (in tick units)
        const val kS = 0.0
        const val kV = 0.0
        const val kA = 0.0

        // path profile parameters (in inches)
        const val maxWheelVel = 50.0
        const val minProfileAccel = -30.0
        const val maxProfileAccel = 50.0

        // turn profile parameters (in radians)
        const val maxAngVel = Math.PI // shared with path

        const val maxAngAccel = Math.PI

        // path controller gains
        const val axialGain = 0.0
        const val lateralGain = 0.0
        const val headingGain = 0.0 // shared with turn

        const val axialVelGain = 0.0
        const val lateralVelGain = 0.0
        const val headingVelGain = 0.0 // shared with turn

    }

    val kinematics = MecanumKinematics(
        Params.inPerTick * Params.trackWidthTicks,
        Params.inPerTick / Params.lateralInPerTick
    )

    private val defaultTurnConstraints = TurnConstraints(
        Params.maxAngVel,
        -Params.maxAngAccel,
        Params.maxAngAccel
    )
    private val defaultVelConstraint: VelConstraint = MinVelConstraint(
        listOf(
            kinematics.WheelVelConstraint(Params.maxWheelVel),
            AngularVelConstraint(Params.maxAngVel)
        )
    )
    private val defaultAccelConstraint: AccelConstraint = ProfileAccelConstraint(
        Params.minProfileAccel,
        Params.maxProfileAccel
    )


    private val localizer = DriveLocalizer()

    var pose = initPose2d
    private val poseHistory = LinkedList<Pose2d>()

    val leftFront = hardwareMap.get<DcMotorEx>("leftFront")
    val leftBack = hardwareMap.get<DcMotorEx>("leftBack")
    val rightBack = hardwareMap.get<DcMotorEx>("rightBack")
    val rightFront = hardwareMap.get<DcMotorEx>("rightFront")

    private val motors = listOf(leftFront, leftBack, rightBack, rightFront)

    val imu = hardwareMap.get<IMU>("imu")

    val voltageSensor: VoltageSensor = hardwareMap.voltageSensor.iterator().next()

    inner class DriveLocalizer : Localizer {
        private val leftFrontEnc = OverflowEncoder(RawEncoder(leftFront))
        private val leftBackEnc = OverflowEncoder(RawEncoder(leftBack))
        private val rightBackEnc = OverflowEncoder(RawEncoder(rightBack))
        private val rightFrontEnc = OverflowEncoder(RawEncoder(rightFront))

        private var lastLeftFrontPos = leftFrontEnc.getPositionAndVelocity().position
        private var lastLeftBackPos = leftBackEnc.getPositionAndVelocity().position
        private var lastRightBackPos = rightBackEnc.getPositionAndVelocity().position
        private var lastRightFrontPos = rightFrontEnc.getPositionAndVelocity().position

        private var lastHeading = Rotation2d.exp(imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS))

        override fun update(): Twist2dDual<Time> {
            val leftFrontPosVel = leftFrontEnc.getPositionAndVelocity()
            val leftBackPosVel = leftBackEnc.getPositionAndVelocity()
            val rightBackPosVel = rightBackEnc.getPositionAndVelocity()
            val rightFrontPosVel = rightFrontEnc.getPositionAndVelocity()

            val heading = Rotation2d.exp(imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS))

            val headingDelta = heading.minus(lastHeading)

            val twist = kinematics.forward(
                MecanumKinematics.WheelIncrements(
                    DualNum<Time>(
                        listOf(
                            (leftFrontPosVel.position - lastLeftFrontPos).toDouble(),
                            leftFrontPosVel.velocity.toDouble()
                        )
                    ).times(Params.inPerTick),
                    DualNum<Time>(
                        listOf(
                            (leftBackPosVel.position - lastLeftBackPos).toDouble(),
                            leftBackPosVel.velocity.toDouble()
                        )
                    ).times(Params.inPerTick),
                    DualNum<Time>(
                        listOf(
                            (rightBackPosVel.position - lastRightBackPos).toDouble(),
                            rightBackPosVel.velocity.toDouble()
                        )
                    ).times(Params.inPerTick),
                    DualNum<Time>(
                        listOf(
                            (rightFrontPosVel.position - lastRightFrontPos).toDouble(),
                            rightFrontPosVel.velocity.toDouble()
                        )
                    ).times(Params.inPerTick)
                )
            )

            lastLeftFrontPos = leftFrontPosVel.position
            lastLeftBackPos = leftBackPosVel.position
            lastRightBackPos = rightBackPosVel.position
            lastRightFrontPos = rightFrontPosVel.position

            lastHeading = heading

            return Twist2dDual(
                twist.line,
                DualNum.cons(headingDelta, twist.angle.drop(1))
            )
        }

    }

    init {
        throwIfModulesAreOutdated(hardwareMap)

        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        motors.forEach {
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        leftBack.direction = DcMotorSimple.Direction.REVERSE
        leftFront.direction = DcMotorSimple.Direction.REVERSE

        val parameters = IMU.Parameters(RevHubOrientationOnRobot(
            Params.logoFacingDirection,
            Params.usbFacingDirection
        ))

        imu.initialize(parameters)
    }

    fun setDrivePowers(powers: PoseVelocity2d?) {
        val wheelVels: MecanumKinematics.WheelVelocities<Time> = MecanumKinematics(1.0).inverse(
            PoseVelocity2dDual.constant(powers!!, 1)
        )
        var maxPowerMag = 1.0
        for (power in wheelVels.all()) {
            maxPowerMag = maxPowerMag.coerceAtLeast(power.value())
        }
        leftFront.power = wheelVels.leftFront[0] / maxPowerMag
        leftBack.power = wheelVels.leftBack[0] / maxPowerMag
        rightBack.power = wheelVels.rightBack[0] / maxPowerMag
        rightFront.power = wheelVels.rightFront[0] / maxPowerMag
    }


    inner class FollowTrajectoryAction(private val timeTrajectory: TimeTrajectory) : Action {
        private var beginTs = -1.0
        private val xPoints: DoubleArray
        private val yPoints: DoubleArray

        init {
            val disps = range(
                0.0, timeTrajectory.path.length(),
                2.coerceAtLeast(ceil(timeTrajectory.path.length() / 2).toInt())
            )
            xPoints = DoubleArray(disps.size)
            yPoints = DoubleArray(disps.size)
            for (i in disps.indices) {
                val (position) = timeTrajectory.path[disps[i], 1].value()
                xPoints[i] = position.x
                yPoints[i] = position.y
            }
        }

        override fun run(p: TelemetryPacket): Boolean {
            val t: Double
            if (beginTs < 0) {
                beginTs = now()
                t = 0.0
            } else {
                t = now() - beginTs
            }
            if (t >= timeTrajectory.duration) {
                leftFront.power = 0.0
                leftBack.power = 0.0
                rightBack.power = 0.0
                rightFront.power = 0.0
                return false
            }
            val txWorldTarget = timeTrajectory[t]
            val robotVelRobot: PoseVelocity2d = updatePoseEstimate()
            val command = HolonomicController(
                Params.axialGain,
                Params.lateralGain,
                Params.headingGain,
                Params.axialVelGain,
                Params.lateralVelGain,
                Params.headingVelGain
            ).compute(txWorldTarget, pose, robotVelRobot)

            val wheelVels: MecanumKinematics.WheelVelocities<Time> = kinematics.inverse(command)
            val voltage: Double = voltageSensor.voltage
            val feedforward = MotorFeedforward(
                Params.kS,
                Params.kV / Params.inPerTick,
                Params.kA / Params.inPerTick
            )
            val leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage
            val leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage
            val rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage
            val rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage
            leftFront.power = leftFrontPower
            leftBack.power = leftBackPower
            rightBack.power = rightBackPower
            rightFront.power = rightFrontPower
            p.put("x", pose.position.x)
            p.put("y", pose.position.y)
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()))
            val (position, heading) = txWorldTarget.value().minusExp(pose)
            p.put("xError", position.x)
            p.put("yError", position.y)
            p.put("headingError (deg)", Math.toDegrees(heading.toDouble()))

            // only draw when active; only one drive action should be active at a time
            val c = p.fieldOverlay()
            drawPoseHistory(c)
            c.setStroke("#4CAF50")
            drawRobot(c, txWorldTarget.value())
            c.setStroke("#3F51B5")
            drawRobot(c, pose)
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


    inner class TurnAction(private val turn: TimeTurn) : Action {
        private var beginTs = -1.0
        override fun run(p: TelemetryPacket): Boolean {
            val t: Double
            if (beginTs < 0) {
                beginTs = now()
                t = 0.0
            } else {
                t = now() - beginTs
            }
            if (t >= turn.duration) {
                leftFront.power = 0.0
                leftBack.power = 0.0
                rightBack.power = 0.0
                rightFront.power = 0.0
                return false
            }
            val txWorldTarget = turn[t]
            val robotVelRobot: PoseVelocity2d = updatePoseEstimate()
            val command = HolonomicController(
                Params.axialGain,
                Params.lateralGain,
                Params.headingGain,
                Params.axialVelGain,
                Params.lateralVelGain,
                Params.headingVelGain
            ).compute(txWorldTarget, pose, robotVelRobot)
            val wheelVels: MecanumKinematics.WheelVelocities<Time> = kinematics.inverse(command)
            val voltage: Double = voltageSensor.voltage
            val feedforward = MotorFeedforward(
                Params.kS,
                Params.kV / Params.inPerTick,
                Params.kA / Params.inPerTick
            )
            val leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage
            val leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage
            val rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage
            val rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage
            leftFront.power = leftFrontPower
            leftBack.power = leftBackPower
            rightBack.power = rightBackPower
            rightFront.power = rightFrontPower
            val c = p.fieldOverlay()
            drawPoseHistory(c)
            c.setStroke("#4CAF50")
            drawRobot(c, txWorldTarget.value())
            c.setStroke("#3F51B5")
            drawRobot(c, pose)
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
        pose = pose.plus(twist.value())
        poseHistory.add(pose)
        while (poseHistory.size > 100) {
            poseHistory.removeFirst()
        }
        return twist.velocity().value()
    }

    private fun drawPoseHistory(c: Canvas) {
        val xPoints = DoubleArray(poseHistory.size)
        val yPoints = DoubleArray(poseHistory.size)
        var i = 0
        for ((position) in poseHistory) {
            xPoints[i] = position.x
            yPoints[i] = position.y
            i++
        }
        c.setStrokeWidth(1)
        c.setStroke("#3F51B5")
        c.strokePolyline(xPoints, yPoints)
    }

    private fun drawRobot(c: Canvas, t: Pose2d) {
        val robotRadius = 9.0
        c.setStrokeWidth(1)
        c.strokeCircle(t.position.x, t.position.y, robotRadius)
        val halfv = t.heading.vec().times(0.5 * robotRadius)
        val p1 = t.position.plus(halfv)
        val (x, y) = p1.plus(halfv)
        c.strokeLine(p1.x, p1.y, x, y)
    }

    fun actionBuilder(beginPose: Pose2d): TrajectoryActionBuilder {
        return TrajectoryActionBuilder(
            { TurnAction(it) },
            { FollowTrajectoryAction(it) },
            beginPose, 1e-6, 0.0,
            defaultTurnConstraints,
            defaultVelConstraint, defaultAccelConstraint,
            0.25, 0.1
        )
    }

    fun actionBuilder(beginPose: Pose): TrajectoryActionBuilder {
        return actionBuilder(beginPose.toPose2d())
    }
}