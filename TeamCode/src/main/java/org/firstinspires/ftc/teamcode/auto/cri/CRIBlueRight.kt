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
class CRIBlueRight : MultiThreadOpMode() {

    private val avgWindow = 100

    private val startPose = Pose(-2.5.tile, 2.5.tile + 3.inch, 90.deg)

    private val midPurplePixel = Pose(-2.5.tile, 0.5.tile + 4.inch, 90.deg)
    private val rightPurplePixel = Pose(-3.tile, 1.tile, -90.deg)
    private val leftPurplePixel = Pose(2.5.tile - 5.inch, -1.5.tile, 0.deg)

    private val midStacky1 = Pose(-3.4.tile, 0.5.tile, 180.deg)
    private val leftStacky1 = Pose(-3.4.tile, 0.5.tile, 180.deg)
    private val rightStacky1 = Pose(-3.4.tile, 0.5.tile, 180.deg)

    private val midStacky2 = Pose(-3.3.tile, 0.5.tile, 180.deg)
    private val leftStacky2 = Pose(-3.3.tile, 0.5.tile, 180.deg)
    private val rightStacky2 = Pose(-3.3.tile, 0.5.tile, 180.deg)

    private val midTransition1 = Pose(2.tile, 0.5.tile, 180.deg)
    private val leftTransition1 = Pose(2.tile, 0.5.tile, 180.deg)
    private val rightTransition1 = Pose(2.tile, 0.5.tile, 180.deg)

    private val midTransition2 = Pose(2.tile, 0.5.tile, 180.deg)
    private val leftTransition2 = Pose(2.tile, 0.5.tile, 180.deg)
    private val rightTransition2 = Pose(2.tile, 0.5.tile, 180.deg)

    private val midPreTruss1 = Pose(0.tile, 0.5.tile, 180.deg)
    private val leftPreTruss1 = Pose(0.tile, 0.5.tile, 180.deg)
    private val rightPreTruss1 = Pose(0.tile, 0.5.tile, 180.deg)

    private val midPreTruss2 = Pose(0.tile, 0.5.tile, 180.deg)
    private val leftPreTruss2 = Pose(0.tile, 0.5.tile, 180.deg)
    private val rightPreTruss2 = Pose(0.tile, 0.5.tile, 180.deg)

    private val midPostTruss1 = Pose(1.tile, 0.5.tile, 180.deg)
    private val leftPostTruss1 = Pose(1.tile, 0.5.tile, 180.deg)
    private val rightPostTruss1 = Pose(1.tile, 0.5.tile, 180.deg)

    private val midPostTruss2 = Pose(1.tile, 0.5.tile, 180.deg)
    private val leftPostTruss2 = Pose(1.tile, 0.5.tile, 180.deg)
    private val rightPostTruss2 = Pose(1.tile, 0.5.tile, 180.deg)

    private val midCenterBoard = Pose(3.tile + 3.inch, 1.5.tile, 180.deg)
    private val midRightBoard1 = Pose(3.tile + 3.inch, 1.5.tile - 6.inch, 180.deg)
    private val midRightBoard2 = Pose(3.tile + 3.inch, 1.5.tile - 6.inch, 180.deg)

    private val leftLeftBoard = Pose(3.tile + 3.inch, 1.5.tile + 6.inch, 180.deg)
    private val leftRightBoard1 = Pose(3.tile + 3.inch, 1.5.tile - 6.inch, 180.deg)
    private val leftRightBoard2 = Pose(3.tile + 3.inch, 1.5.tile - 6.inch, 180.deg)

    private val rightRightBoard = Pose(3.tile + 3.inch, 1.5.tile - 6.inch, 180.deg)
    private val rightCenterBoard1 = Pose(3.tile + 3.inch, 1.5.tile, 180.deg)
    private val rightCenterBoard2 = Pose(3.tile + 3.inch, 1.5.tile, 180.deg)

    private val parkReverseDistance = 10.cm

    private val parkY = 0.5.tile

    private val midBoardAproachAngle = 30.deg
    private val leftBoardAproachAngle = midBoardAproachAngle
    private val rightBoardAproachAngle = midBoardAproachAngle

    private val midBoardLeavingAngle = 210.deg
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
        camera.setColor(ColorVisionProcessor.DetectionColor.BLUE)

        fun enableApril() = InstantAction { drive.useApril = true }
        fun disableApril() = InstantAction { drive.useApril = false }

        val actionLeft = SequentialAction(
            drive.actionBuilder(startPose)
                .setTangent(225.deg)
                .splineToLinearHeading(rightPurplePixel, 0.deg)
                .stopAndAdd(intake.ejectPurple())
                .setTangent(180.deg)
                .splineTo(rightPurplePixel.position - 4.cm.x, 180.deg)
                .afterTime(0.s, firstStackPrep())
                .splineToLinearHeading(rightStacky1, 180.deg)
                .build(),
            drive.CorrectionAction(leftStacky1),
            InstantAction { intake.firstStack() },
            intake.waitForPixel(),
            InstantAction { box.power = 0.0 },
            drive.actionBuilder(leftStacky1)
                .setTangent(0.deg)
                .afterTime(0.s, intake.ejectPixels())
                .splineToConstantHeading(leftPreTruss1.position, 0.deg)
                .splineToConstantHeading(leftPostTruss1.position, 0.deg, speed20)
                .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToAboveWhite())
                .splineToConstantHeading(leftTransition1.position, 0.deg)
                .splineToConstantHeading(leftRightBoard1.position, leftBoardAproachAngle, speed60)
                .build(),
            drive.CorrectionAction(leftRightBoard1),
            enableApril(),
            ejectTillYellow(),
            disableApril(),
            drive.actionBuilder(leftRightBoard1)
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
                .splineToConstantHeading(leftTransition2.position, 180.deg, speed60)
                .splineToConstantHeading(leftPostTruss2.position, 180.deg)
                .splineToConstantHeading(leftPreTruss2.position, 180.deg, speed20)
                .afterTime(0.s, secondStackPrep())
                .splineToConstantHeading(leftStacky2.position, 180.deg)
                .build(),
            drive.CorrectionAction(leftStacky2),
            InstantAction { intake.secondStack() },
            intake.waitForPixel(),
            InstantAction { box.power = 0.0 },
            drive.actionBuilder(leftStacky2)
                .setTangent(0.deg)
                .afterTime(0.s, intake.ejectPixels())
                .splineToConstantHeading(leftPreTruss2.position, 0.deg)
                .splineToConstantHeading(leftPostTruss2.position, 0.deg, speed20)
                .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToAboveWhite())
                .splineToConstantHeading(leftTransition2.position, 0.deg)
                .splineToConstantHeading(leftRightBoard2.position, leftBoardAproachAngle, speed60)
                .build(),
            drive.CorrectionAction(leftRightBoard2),
            box.ejectTwoPixels(),
            drive.actionBuilder(leftRightBoard2)
                .setTangent(180.deg)
                .lineToX(leftRightBoard2.position.x - parkReverseDistance)
                .afterTime(0.s, systemsToIntake())
                .setTangent(90.deg)
                .lineToY(parkY)
                .build()
        )

        val actionMiddle = SequentialAction(
            drive.actionBuilder(startPose)
                .strafeToLinearHeading(midPurplePixel.position, midPurplePixel.heading)
                .stopAndAdd(intake.ejectPurple())
                .setTangent(-90.deg)
                .afterTime(0.s, firstStackPrep())
                .splineToLinearHeading(midStacky1, 180.deg)
                .build(),
            drive.CorrectionAction(midStacky1),
            InstantAction { intake.firstStack() },
            intake.waitForPixel(),
            InstantAction { box.power = 0.0 },
            drive.actionBuilder(midStacky1)
                .setTangent(0.deg)
                .afterTime(0.s, intake.ejectPixels())
                .splineToConstantHeading(midPreTruss1.position, 0.deg)
                .splineToConstantHeading(midPostTruss1.position, 0.deg, speed20)
                .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToAboveWhite())
                .splineToConstantHeading(midTransition1.position, 0.deg)
                .splineToConstantHeading(midRightBoard1.position, midBoardAproachAngle, speed60)
                .build(),
            drive.CorrectionAction(midRightBoard1),
            enableApril(),
            ejectTillYellow(),
            disableApril(),
            drive.actionBuilder(midRightBoard1)
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
                .splineToConstantHeading(midTransition2.position, 180.deg, speed60)
                .splineToConstantHeading(midPostTruss2.position, 180.deg)
                .splineToConstantHeading(midPreTruss2.position, 180.deg, speed20)
                .afterTime(0.s, secondStackPrep())
                .splineToConstantHeading(midStacky2.position, 180.deg)
                .build(),
            drive.CorrectionAction(midStacky2),
            InstantAction { intake.secondStack() },
            intake.waitForPixel(),
            InstantAction { box.power = 0.0 },
            drive.actionBuilder(midStacky2)
                .setTangent(0.deg)
                .afterTime(0.s, intake.ejectPixels())
                .splineToConstantHeading(midPreTruss2.position, 0.deg)
                .splineToConstantHeading(midPostTruss2.position, 0.deg, speed20)
                .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToAboveWhite())
                .splineToConstantHeading(midTransition2.position, 0.deg)
                .splineToConstantHeading(midRightBoard2.position, midBoardAproachAngle, speed60)
                .build(),
            drive.CorrectionAction(midRightBoard2),
            box.ejectTwoPixels(),
            drive.actionBuilder(midRightBoard2)
                .setTangent(180.deg)
                .lineToX(midRightBoard2.position.x - parkReverseDistance)
                .afterTime(0.s, systemsToIntake())
                .setTangent(90.deg)
                .lineToY(parkY)
                .build()
        )

        val actionRight = SequentialAction(
            drive.actionBuilder(startPose)
                .strafeToLinearHeading(leftPurplePixel.position, leftPurplePixel.heading)
                .stopAndAdd(intake.ejectPurple())
                .setTangent(-90.deg)
                .afterTime(0.s, firstStackPrep())
                .splineToLinearHeading(leftStacky1, 180.deg)
                .build(),
            drive.CorrectionAction(rightStacky1),
            InstantAction { intake.firstStack() },
            intake.waitForPixel(),
            InstantAction { box.power = 0.0 },
            drive.actionBuilder(rightStacky1)
                .setTangent(0.deg)
                .afterTime(0.s, intake.ejectPixels())
                .splineToConstantHeading(rightPreTruss1.position, 0.deg)
                .splineToConstantHeading(rightPostTruss1.position, 0.deg, speed20)
                .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToAboveWhite())
                .splineToConstantHeading(rightTransition1.position, 0.deg)
                .splineToConstantHeading(rightCenterBoard1.position, rightBoardAproachAngle, speed60)
                .build(),
            drive.CorrectionAction(rightCenterBoard1),
            enableApril(),
            ejectTillYellow(),
            disableApril(),
            drive.actionBuilder(rightCenterBoard1)
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
                .splineToConstantHeading(rightTransition2.position, 180.deg, speed60)
                .splineToConstantHeading(rightPostTruss2.position, 180.deg)
                .splineToConstantHeading(rightPreTruss2.position, 180.deg, speed20)
                .afterTime(0.s, secondStackPrep())
                .splineToConstantHeading(rightStacky2.position, 180.deg)
                .build(),
            drive.CorrectionAction(rightStacky2),
            InstantAction { intake.secondStack() },
            intake.waitForPixel(),
            InstantAction { box.power = 0.0 },
            drive.actionBuilder(rightStacky2)
                .setTangent(0.deg)
                .afterTime(0.s, intake.ejectPixels())
                .splineToConstantHeading(rightPreTruss2.position, 0.deg)
                .splineToConstantHeading(rightPostTruss2.position, 0.deg, speed20)
                .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToAboveWhite())
                .splineToConstantHeading(rightTransition2.position, 0.deg)
                .splineToConstantHeading(rightCenterBoard2.position, rightBoardAproachAngle, speed60)
                .build(),
            drive.CorrectionAction(rightCenterBoard2),
            box.ejectTwoPixels(),
            drive.actionBuilder(rightCenterBoard2)
                .setTangent(180.deg)
                .lineToX(rightCenterBoard2.position.x - parkReverseDistance)
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