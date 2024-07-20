package org.firstinspires.ftc.teamcode.auto.cri

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.InstantFunction
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.outoftheboxrobotics.photoncore.Photon
import com.phoenix.phoenixlib.units.Pose
import com.phoenix.phoenixlib.units.Time
import com.phoenix.phoenixlib.units.cm
import com.phoenix.phoenixlib.units.deg
import com.phoenix.phoenixlib.units.inch
import com.phoenix.phoenixlib.units.ms
import com.phoenix.phoenixlib.units.s
import com.phoenix.phoenixlib.units.tile
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.lib.hardware.controlHub
import org.firstinspires.ftc.teamcode.lib.hardware.expansionHub
import org.firstinspires.ftc.teamcode.lib.opmode.MultiThreadOpMode
import org.firstinspires.ftc.teamcode.lib.vision.ColorVisionProcessor
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.systems.Camera
import org.firstinspires.ftc.teamcode.systems.Lift
import org.firstinspires.ftc.teamcode.systems.multi.ArmMulti.Companion.armMulti
import org.firstinspires.ftc.teamcode.systems.multi.BoxMulti.Companion.boxMulti
import org.firstinspires.ftc.teamcode.systems.multi.Color2Multi.Companion.color2Multi
import org.firstinspires.ftc.teamcode.systems.multi.IntakeMulti.Companion.intakeMulti
import org.firstinspires.ftc.teamcode.systems.multi.LiftMulti.Companion.liftMulti
import java.util.ArrayDeque

@Photon
@Autonomous(preselectTeleOp = "LammaDriveRed", group = "CRI")
class CRIRedLeft : MultiThreadOpMode() {

    private val avgWindow = 100

    private val startPose = Pose(-2.5.tile, -2.5.tile - 1.inch, -90.deg)

    private val midPurplePixel = Pose(-2.5.tile, -0.5.tile - 4.inch, -90.deg)
    private val leftPurplePixel = Pose(-3.tile, -1.tile, -90.deg)
    private val rightPurplePixel = Pose(-2.5.tile + 5.inch, -1.5.tile, 0.deg)

    private val midStacky = Pose(-3.5.tile, -0.5.tile, 180.deg)
    private val leftStacky = midStacky
    private val rightStacky = midStacky

    private val midTransition = Pose(2.tile, -0.5.tile, 180.deg)
    private val leftTransition = midTransition
    private val rightTransition = midTransition

    private val midPreTruss = Pose(0.tile, -0.5.tile, 180.deg)
    private val leftPreTruss = midPreTruss
    private val rightPreTruss = midPreTruss

    private val midPostTruss = Pose(1.tile, -0.5.tile, 180.deg)
    private val leftPostTruss = midPostTruss
    private val rightPostTruss = midPostTruss

    private val midCenterBoard = Pose(3.tile + 3.inch, -1.5.tile, 180.deg)
    private val midLeftBoard = Pose(3.tile + 3.inch, -1.5.tile + 6.inch, 180.deg)

    private val leftLeftBoard = Pose(3.tile + 3.inch, -1.5.tile + 6.inch, 180.deg)
    private val leftCenterBoard = Pose(3.tile + 3.inch, -1.5.tile, 180.deg)

    private val rightRightBoard = Pose(3.tile + 3.inch, -1.5.tile - 6.inch, 180.deg)
    private val rightLeftBoard = Pose(3.tile + 3.inch, -1.5.tile + 6.inch, 180.deg)

    private val parkReverseDistance = 10.cm

    private val parkY = -0.5.tile

    private val midBoardAproachAngle = -30.deg
    private val leftBoardAproachAngle = midBoardAproachAngle
    private val rightBoardAproachAngle = midBoardAproachAngle

    private val midBoardLeavingAngle = 150.deg
    private val leftBoardLeavingAngle = midBoardLeavingAngle
    private val rightBoardLeavingAngle = midBoardLeavingAngle
    
    private val drive by opModeLazy {
        MecanumDrive(hardwareMap, startPose.pose2d)
    }

    private val arm by opModeLazy {
        hardwareMap.armMulti()
    }

    private val box by opModeLazy {
        hardwareMap.boxMulti()
    }

    private val lift by opModeLazy {
        hardwareMap.liftMulti()
    }

    private val intake by opModeLazy {
        hardwareMap.intakeMulti()
    }

    private val camera by opModeLazy {
        Camera(hardwareMap)
    }

    private val color by opModeLazy {
        hardwareMap.color2Multi()
    }

    private var sideDeltaTime = 20.ms

    override fun sideRunOpMode() {
        var previousTime = Time.now()

        val expansionHub = hardwareMap.expansionHub()

        expansionHub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL

        while (opModeInInit()) {
            camera.update()
        }

        while (isStarted && !isStopRequested) {
            val now = Time.now()
            sideDeltaTime = now - previousTime
            previousTime = now

            expansionHub.clearBulkCache()

            camera.update()
            arm.write()
            arm.update()
            box.write()
            box.update()
            lift.read()
            lift.write()
            lift.update()
            intake.write()
            intake.update()
            color.read()
        }
    }

    override fun mainRunOpMode() {

        val speed60 = MinVelConstraint(listOf(
            drive.kinematics.WheelVelConstraint(60.0),
            AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel)
        ))
        val speed20 = MinVelConstraint(listOf(
            drive.kinematics.WheelVelConstraint(20.0),
            AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel)
        ))

        var previousTime = Time.now()
        var mainDeltaTime: Time

        val controlHub = hardwareMap.controlHub()

        controlHub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL

        val dash = FtcDashboard.getInstance()

        telemetry = MultipleTelemetry(telemetry, dash.telemetry)

        drive.camera = camera
        camera.telemetry = telemetry
        camera.setColor(ColorVisionProcessor.DetectionColor.RED)

        fun enableApril() = InstantAction { drive.useApril = true }
        fun disableApril() = InstantAction { drive.useApril = false }

        val actionLeft = SequentialAction(
            drive.actionBuilder(startPose)
                .strafeToLinearHeading(leftPurplePixel.position, leftPurplePixel.heading)
                .stopAndAdd(intake.ejectPurple())
                .setTangent(90.deg)
                .afterTime(0.s, firstStackPrep())
                .splineToLinearHeading(leftStacky, 180.deg)
                .build(),
            drive.CorrectionAction(leftStacky),
            InstantAction { intake.firstStack() },
            intake.waitForPixel(),
            InstantAction { box.power = 0.0 },
            drive.actionBuilder(leftStacky)
                .setTangent(0.deg)
                .afterTime(0.s, intake.ejectPixels())
                .splineToConstantHeading(leftPreTruss.position, 0.deg)
                .splineToConstantHeading(leftPostTruss.position, 0.deg, speed20)
                .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToAboveWhite())
                .splineToConstantHeading(leftTransition.position, 0.deg)
                .splineToConstantHeading(leftCenterBoard.position, leftBoardAproachAngle, speed60)
                .build(),
            drive.CorrectionAction(leftCenterBoard),
            enableApril(),
            ejectTillYellow(),
            disableApril(),
            drive.actionBuilder(leftCenterBoard)
                .strafeTo(leftLeftBoard.position)
                .build(),
            drive.CorrectionAction(leftLeftBoard),
            enableApril(),
            lift.goToYellow(),
            box.ejectYellowPixel(),
            InstantAction { lift.targetPositionTicks = Lift.LiftConfig.aboveWhiteTicks },
            disableApril(),
            drive.actionBuilder(leftLeftBoard)
                .setTangent(leftBoardLeavingAngle)
                .afterTime(Lift.LiftConfig.postBoardDecendWaitSec.s, systemsToIntake())
                .splineToConstantHeading(leftTransition.position, 180.deg, speed60)
                .splineToConstantHeading(leftPostTruss.position, 180.deg)
                .splineToConstantHeading(leftPreTruss.position, 180.deg, speed20)
                .afterTime(0.s, secondStackPrep())
                .splineToConstantHeading(leftStacky.position, 180.deg)
                .build(),
            drive.CorrectionAction(leftStacky),
            InstantAction { intake.secondStack() },
            intake.waitForPixel(),
            InstantAction { box.power = 0.0 },
            drive.actionBuilder(leftStacky)
                .setTangent(0.deg)
                .afterTime(0.s, intake.ejectPixels())
                .splineToConstantHeading(leftPreTruss.position, 0.deg)
                .splineToConstantHeading(leftPostTruss.position, 0.deg, speed20)
                .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToAboveWhite())
                .splineToConstantHeading(leftTransition.position, 0.deg)
                .splineToConstantHeading(leftCenterBoard.position, leftBoardAproachAngle, speed60)
                .build(),
            drive.CorrectionAction(leftCenterBoard),
            box.ejectTwoPixels(),
            drive.actionBuilder(leftCenterBoard)
                .setTangent(180.deg)
                .lineToX(leftCenterBoard.position.x - parkReverseDistance)
                .afterTime(0.s, systemsToIntake())
                .setTangent(90.deg)
                .lineToY(parkY)
                .build()
        )

        val actionMiddle = SequentialAction(
            drive.actionBuilder(startPose)
                .strafeToLinearHeading(midPurplePixel.position, midPurplePixel.heading)
                .stopAndAdd(intake.ejectPurple())
                .setTangent(90.deg)
                .afterTime(0.s, firstStackPrep())
                .splineToLinearHeading(midStacky, 180.deg)
                .build(),
            drive.CorrectionAction(midStacky),
            InstantAction { intake.firstStack() },
            intake.waitForPixel(),
            InstantAction { box.power = 0.0 },
            drive.actionBuilder(midStacky)
                .setTangent(0.deg)
                .afterTime(0.s, intake.ejectPixels())
                .splineToConstantHeading(midPreTruss.position, 0.deg)
                .splineToConstantHeading(midPostTruss.position, 0.deg, speed20)
                .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToAboveWhite())
                .splineToConstantHeading(midTransition.position, 0.deg)
                .splineToConstantHeading(midLeftBoard.position, midBoardAproachAngle, speed60)
                .build(),
            drive.CorrectionAction(midLeftBoard),
            enableApril(),
            ejectTillYellow(),
            disableApril(),
            drive.actionBuilder(midLeftBoard)
                .strafeTo(midCenterBoard.position)
                .build(),
            drive.CorrectionAction(midCenterBoard),
            enableApril(),
            lift.goToYellow(),
            box.ejectYellowPixel(),
            InstantAction { lift.targetPositionTicks = Lift.LiftConfig.aboveWhiteTicks },
            disableApril(),
            drive.actionBuilder(midCenterBoard)
                .setTangent(midBoardLeavingAngle)
                .afterTime(Lift.LiftConfig.postBoardDecendWaitSec.s, systemsToIntake())
                .splineToConstantHeading(midTransition.position, 180.deg, speed60)
                .splineToConstantHeading(midPostTruss.position, 180.deg)
                .splineToConstantHeading(midPreTruss.position, 180.deg, speed20)
                .afterTime(0.s, secondStackPrep())
                .splineToConstantHeading(midStacky.position, 180.deg)
                .build(),
            drive.CorrectionAction(midStacky),
            InstantAction { intake.secondStack() },
            intake.waitForPixel(),
            InstantAction { box.power = 0.0 },
            drive.actionBuilder(midStacky)
                .setTangent(0.deg)
                .afterTime(0.s, intake.ejectPixels())
                .splineToConstantHeading(midPreTruss.position, 0.deg)
                .splineToConstantHeading(midPostTruss.position, 0.deg, speed20)
                .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToAboveWhite())
                .splineToConstantHeading(midTransition.position, 0.deg)
                .splineToConstantHeading(midLeftBoard.position, midBoardAproachAngle, speed60)
                .build(),
            drive.CorrectionAction(midLeftBoard),
            box.ejectTwoPixels(),
            drive.actionBuilder(midLeftBoard)
                .setTangent(180.deg)
                .lineToX(midLeftBoard.position.x - parkReverseDistance)
                .afterTime(0.s, systemsToIntake())
                .setTangent(90.deg)
                .lineToY(parkY)
                .build()
        )

        val actionRight = SequentialAction(
            drive.actionBuilder(startPose)
                .setTangent(135.deg)
                .splineToLinearHeading(rightPurplePixel, 0.deg)
                .stopAndAdd(intake.ejectPurple())
                .setTangent(180.deg)
                .splineTo(rightPurplePixel.position - 4.cm.x, 180.deg)
                .afterTime(0.s, firstStackPrep())
                .splineToLinearHeading(rightStacky, 180.deg)
                .build(),
            drive.CorrectionAction(rightStacky),
            InstantAction { intake.firstStack() },
            intake.waitForPixel(),
            InstantAction { box.power = 0.0 },
            drive.actionBuilder(rightStacky)
                .setTangent(0.deg)
                .afterTime(0.s, intake.ejectPixels())
                .splineToConstantHeading(rightPreTruss.position, 0.deg)
                .splineToConstantHeading(rightPostTruss.position, 0.deg, speed20)
                .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToAboveWhite())
                .splineToConstantHeading(rightTransition.position, 0.deg)
                .splineToConstantHeading(rightLeftBoard.position, rightBoardAproachAngle, speed60)
                .build(),
            drive.CorrectionAction(rightLeftBoard),
            enableApril(),
            ejectTillYellow(),
            disableApril(),
            drive.actionBuilder(rightLeftBoard)
                .strafeTo(rightRightBoard.position)
                .build(),
            drive.CorrectionAction(rightRightBoard),
            enableApril(),
            lift.goToYellow(),
            box.ejectYellowPixel(),
            InstantAction { lift.targetPositionTicks = Lift.LiftConfig.aboveWhiteTicks },
            disableApril(),
            drive.actionBuilder(rightRightBoard)
                .setTangent(rightBoardLeavingAngle)
                .afterTime(Lift.LiftConfig.postBoardDecendWaitSec.s, systemsToIntake())
                .splineToConstantHeading(rightTransition.position, 180.deg, speed60)
                .splineToConstantHeading(rightPostTruss.position, 180.deg)
                .splineToConstantHeading(rightPreTruss.position, 180.deg, speed20)
                .afterTime(0.s, secondStackPrep())
                .splineToConstantHeading(rightStacky.position, 180.deg)
                .build(),
            drive.CorrectionAction(rightStacky),
            InstantAction { intake.secondStack() },
            intake.waitForPixel(),
            InstantAction { box.power = 0.0 },
            drive.actionBuilder(rightStacky)
                .setTangent(0.deg)
                .afterTime(0.s, intake.ejectPixels())
                .splineToConstantHeading(rightPreTruss.position, 0.deg)
                .splineToConstantHeading(rightPostTruss.position, 0.deg, speed20)
                .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToAboveWhite())
                .splineToConstantHeading(rightTransition.position, 0.deg)
                .splineToConstantHeading(rightLeftBoard.position, rightBoardAproachAngle, speed60)
                .build(),
            drive.CorrectionAction(rightLeftBoard),
            box.ejectTwoPixels(),
            drive.actionBuilder(rightLeftBoard)
                .setTangent(180.deg)
                .lineToX(rightLeftBoard.position.x - parkReverseDistance)
                .afterTime(0.s, systemsToIntake())
                .setTangent(90.deg)
                .lineToY(parkY)
                .build()
        )

        while (opModeInInit()) {
            camera.displayDetection()
            telemetry.update()
            sleep(10)
        }

        //val action = actionMiddle

        val action = when(camera.detectionPosition) {
            ColorVisionProcessor.DetectionPosition.LEFT -> actionLeft
            ColorVisionProcessor.DetectionPosition.CENTER -> actionMiddle
            ColorVisionProcessor.DetectionPosition.RIGHT -> actionRight
        }

        camera.disableColorDetection()
        camera.enableAprilTagDetection()
        camera.lowerExposure()

        val canvas = Canvas()
        action.preview(canvas)

        var running = true

        val avgMainFpsQueue = ArrayDeque<Double>()
        val avgMainTimeQueue = ArrayDeque<Double>()
        val avgSideFpsQueue = ArrayDeque<Double>()
        val avgSideTimeQueue = ArrayDeque<Double>()

        while (isStarted && !isStopRequested && running) {
            val now = Time.now()
            mainDeltaTime = now - previousTime
            previousTime = now

            controlHub.clearBulkCache()

            val packet = TelemetryPacket()
            packet.fieldOverlay().operations.addAll(canvas.operations)

            running = action.run(packet)

            dash.sendTelemetryPacket(packet)

            if (mainDeltaTime.ms >= 5.0) {
                avgMainFpsQueue.push(1.s / mainDeltaTime)
                avgMainTimeQueue.push(mainDeltaTime.ms)
            }

            avgSideFpsQueue.push(1.s / sideDeltaTime)
            avgSideTimeQueue.push(sideDeltaTime.ms)

            while (avgMainFpsQueue.size > avgWindow) {
                avgMainFpsQueue.removeLast()
            }
            while (avgMainTimeQueue.size > avgWindow) {
                avgMainTimeQueue.removeLast()
            }
            while (avgSideFpsQueue.size > avgWindow) {
                avgSideFpsQueue.removeLast()
            }
            while (avgSideTimeQueue.size > avgWindow) {
                avgSideTimeQueue.removeLast()
            }

            telemetry.addData("avg main fps", avgMainFpsQueue.average())
            telemetry.addData("avg main time ms", avgMainTimeQueue.average())
            telemetry.addData("avg side fps", avgSideFpsQueue.average())
            telemetry.addData("avg side time ms", avgSideTimeQueue.average())
            telemetry.addData("lift pos", lift.positionTicks)
            telemetry.addData("lift target pos", lift.targetPositionTicks)
            telemetry.update()
        }
    }

    private fun ejectTillYellow() = SequentialAction(
        InstantAction { box.power = -1.0 },
        color.waitTillYellow(),
        InstantAction { box.power = 0.0 }
    )

    private fun firstStackPrep() = InstantFunction {
        intake.aboveFirstStack()
        intake.stackPower()
        box.power = 1.0
    }

    private fun secondStackPrep() = InstantFunction {
        intake.aboveSecondStack()
        intake.stackPower()
        box.power = 1.0
    }

    private fun systemsToAboveWhite() = SequentialAction(
        lift.goToPass(),
        ParallelAction(
            arm.scorePosQuick(),
            box.scorePosQuick()
        ),
        lift.goToAboveWhite()
    )

    private fun systemsToIntake() = SequentialAction(
        if (lift.positionTicks < Lift.LiftConfig.passTicks)
            lift.goToPass() else InstantAction{},
        ParallelAction(
            arm.intakePosQuick(),
            box.intakePosQuick()
        ),
        InstantAction { box.power = 0.0 },
        lift.goToIntake(),
    )
}