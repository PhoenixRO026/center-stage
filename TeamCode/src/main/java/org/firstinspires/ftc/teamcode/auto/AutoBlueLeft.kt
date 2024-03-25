package org.firstinspires.ftc.teamcode.auto

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
import kotlin.math.min

@Photon
@Autonomous(preselectTeleOp = "LammaDriveBlue")
class AutoBlueLeft : MultiThreadOpMode() {
    private val startPose =             Pose(12.inch, 61.inch, 90.deg)

    private val middlePurplePixel =     Pose(20.inch, 24.inch, 180.deg)
    private val leftPurplePixel =       Pose(31.inch, 32.inch, 180.deg)
    private val rightPurplePixel =      Pose(8.inch, 32.inch, 180.deg)

    //private val middleStacky1 =         Pose(-54.inch - 18.cm, 12.inch, 180.deg)
    //private val rightStacky1 =          Pose(-54.inch - 18.cm, 12.inch, 180.deg)
    //private val leftStacky1 =           Pose(-54.inch - 18.cm, 12.inch, 180.deg)

    //private val middlePreStacky1 =      middleStacky1 + 10.cm.x
    //private val rightPreStacky1 =       rightStacky1 + 10.cm.x
    //private val leftPreStacky1 =        leftStacky1 + 10.cm.x

    //private val middlePostStackRun1 =   Pose(-30.inch, 12.inch - 1.cm, 180.deg)
    //private val rightPostStackRun1 =    middlePostStackRun1
    //private val leftPostStackRun1 =     middlePostStackRun1

    //private val middlePreBoardRun1 =    Pose(18.inch, 12.inch - 1.cm, 180.deg)
    //private val rightPreBoardRun1 =     Pose(18.inch, 12.inch - 1.cm, 180.deg)
    //private val leftPreBoardRun1 =      Pose(18.inch, 12.inch - 1.cm, 180.deg)

    private val middleYellowPixel2 =    Pose(51.inch, 35.inch, 180.deg)
    private val leftYellowPixel2 =      Pose(51.inch, 42.inch, 180.deg)
    private val rightYellowPixel2 =     Pose(51.inch, 30.inch, 180.deg)

    //private val middlePreYellowPixel1 = middleYellowPixel2
    //private val rightPreYellowPixel1 =  rightYellowPixel2
    //private val leftPreYellowPixel1 =   leftYellowPixel2

    private val middlePostBoardRun2 =   Pose(18.inch, 60.inch, 180.deg)
    private val rightPostBoardRun2 =    Pose(18.inch, 60.inch, 180.deg)
    private val leftPostBoardRun2 =     Pose(18.inch, 60.inch, 180.deg)

    private val middlePreStackRun2 =    Pose(-30.inch, 60.inch, 180.deg)
    private val rightPreStackRun2 =     Pose(-30.inch, 60.inch, 180.deg)
    private val leftPreStackRun2 =      Pose(-30.inch, 60.inch, 180.deg)

    private val middleStacky2 =         Pose(-62.inch, 36.inch, 180.deg)
    private val rightStacky2 =          Pose(-62.inch, 36.inch, 180.deg)
    private val leftStacky2 =           Pose(-62.inch, 36.inch, 180.deg)

    private val middlePreStacky2 =      middleStacky2 + 10.cm.x
    private val rightPreStacky2 =       rightStacky2 + 10.cm.x
    private val leftPreStacky2 =        leftStacky2 + 10.cm.x

    private val middlePostStackRun2 =   Pose(-30.inch, 60.inch, 180.deg)
    private val rightPostStackRun2 =    Pose(-30.inch, 60.inch, 180.deg)
    private val leftPostStackRun2 =     Pose(-30.inch, 60.inch, 180.deg)

    private val middlePreBoardRun2 =    Pose(18.inch, 60.inch, 180.deg)
    private val rightPreBoardRun2 =     Pose(18.inch, 60.inch, 180.deg)
    private val leftPreBoardRun2 =      Pose(18.inch, 60.inch, 180.deg)

    private val middleYellowPixel3 =    middleYellowPixel2
    private val rightYellowPixel3 =     rightYellowPixel2
    private val leftYellowPixel3 =      leftYellowPixel2

    private val middlePreYellowPixel2 = Pose(51.inch, 35.inch, 180.deg)
    private val rightPreYellowPixel2 =  Pose(51.inch, 30.inch, 180.deg)
    private val leftPreYellowPixel2 =   Pose(51.inch, 42.inch, 180.deg)

    private val middlePostBoardRun3 =   middlePostBoardRun2
    private val rightPostBoardRun3 =    rightPostBoardRun2
    private val leftPostBoardRun3 =     leftPostBoardRun2

    private val middlePreStackRun3 =    middlePreStackRun2
    private val rightPreStackRun3 =     rightPreStackRun2
    private val leftPreStackRun3 =      leftPreStackRun2

    private val middleStacky3 =         middleStacky2
    private val rightStacky3 =          rightStacky2
    private val leftStacky3 =           leftStacky2

    private val middlePreStacky3 =      middleStacky3 + 10.cm.x
    private val rightPreStacky3 =       rightStacky3 + 10.cm.x
    private val leftPreStacky3 =        leftStacky3 + 10.cm.x

    private val middlePostStackRun3 =   middlePostStackRun2
    private val rightPostStackRun3 =    rightPostStackRun2
    private val leftPostStackRun3 =     leftPostStackRun2

    private val middlePreBoardRun3 =    middlePreBoardRun2
    private val rightPreBoardRun3 =     rightPreBoardRun2
    private val leftPreBoardRun3 =      leftPreBoardRun2

    private val middleYellowPixel4 =    middleYellowPixel3
    private val rightYellowPixel4 =     rightYellowPixel3
    private val leftYellowPixel4 =      leftYellowPixel3

    private val middlePreYellowPixel3 = middlePreYellowPixel2
    private val rightPreYellowPixel3 =  rightPreYellowPixel2
    private val leftPreYellowPixel3 =   leftPreYellowPixel2

    private val park =                  Pose(48.inch, 60.inch, 180.deg)

    private val middleBoardAproachAngle = -30.deg
    private val rightBoardAproachAngle = -30.deg
    private val leftBoardAproachAngle = -30.deg

    private val middleBoardLeavingAngle = 120.deg
    private val rightBoardLeavingAngle = 120.deg
    private val leftBoardLeavingAngle = 120.deg

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
        var previousTime = Time.now()
        var mainDeltaTime: Time

        val controlHub = hardwareMap.controlHub()

        controlHub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL

        val dash = FtcDashboard.getInstance()

        telemetry = MultipleTelemetry(telemetry, dash.telemetry)

        drive.camera = camera
        camera.telemetry = telemetry
        camera.setColor(ColorVisionProcessor.DetectionColor.BLUE)

        val speed60 = MinVelConstraint(listOf(
                drive.kinematics.WheelVelConstraint(60.0),
                AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel)
        ))

        val actionLeft = SequentialAction(
                drive.actionBuilder(startPose)
                        .strafeToLinearHeading(leftPurplePixel.position, leftPurplePixel.heading)
                        .stopAndAdd(intake.ejectPurple())
                        .setTangent(0.deg)
                        .afterTime(0.s, systemsToYellow())
                        .strafeTo(leftYellowPixel2.position)
                        .stopAndAdd(SequentialAction(
                                InstantAction { drive.useApril = true },
                                lift.goToLowYellow(),
                                box.ejectTwoPixels(),
                                lift.goToAboveWhite(),
                                InstantAction { drive.useApril = false }
                        ))
                        .afterTime(Lift.LiftConfig.postBoardDecendWaitSec.s, systemsToIntake())
                        .setTangent(leftBoardLeavingAngle)
                        .splineToConstantHeading(leftPostBoardRun2.position, 180.deg, speed60)
                        .splineToConstantHeading(leftPreStackRun2.position, 180.deg)
                        .afterTime(0.s, firstStackPrep())
                        .splineToConstantHeading(leftStacky2.position, 180.deg, speed60)
                        .build(),
                drive.CorrectionAction(leftStacky2, MecanumDrive.PARAMS.maxStackCorrectTimeSec.s),
                InstantAction { intake.boardSideFirstStack() },
                intake.waitForPixel(),
                InstantAction { box.power = 0.0 },
                drive.actionBuilder(leftStacky2)
                        .afterTime(0.s, intake.ejectPixels())
                        .setTangent(0.deg)
                        .splineToConstantHeading(leftPostStackRun2.position, 0.deg, speed60)
                        .afterTime(Lift.LiftConfig.boardSidePostStackRiseWaitSec.s, systemsToAboveWhite())
                        .splineToConstantHeading(leftPreBoardRun2.position, 0.deg)
                        .splineToConstantHeading(middlePreYellowPixel2.position, middleBoardAproachAngle, speed60)
                        .build(),
                InstantAction { drive.useApril = true },
                drive.CorrectionAction(middleYellowPixel3, MecanumDrive.PARAMS.maxStackCorrectTimeSec.s),
                box.ejectTwoPixels(),
                InstantAction { drive.useApril = false },
                drive.actionBuilder(middleYellowPixel3)
                        .afterTime(Lift.LiftConfig.postBoardDecendWaitSec.s, systemsToIntake())
                        .setTangent(middleBoardLeavingAngle)
                        .splineToConstantHeading(leftPostBoardRun3.position, 180.deg, speed60)
                        .splineToConstantHeading(leftPreStackRun3.position, 180.deg)
                        .afterTime(0.s, secondStackPrep())
                        .splineToConstantHeading(leftStacky3.position, 180.deg)
                        .build(),
                drive.CorrectionAction(leftStacky3, MecanumDrive.PARAMS.maxStackCorrectTimeSec.s),
                InstantAction { intake.boardSideSecondStack() },
                intake.waitForPixel(),
                InstantAction { box.power = 0.0 },
                drive.actionBuilder(leftStacky3)
                        .afterTime(0.s, intake.ejectPixels())
                        .setTangent(0.deg)
                        .splineToConstantHeading(leftPostStackRun3.position, 0.deg, speed60)
                        .afterTime(Lift.LiftConfig.boardSidePostStackRiseWaitSec.s, systemsToAboveWhite())
                        .splineToConstantHeading(leftPreBoardRun3.position, 0.deg)
                        .splineToConstantHeading(middlePreYellowPixel3.position, middleBoardAproachAngle, speed60)
                        .build(),
                InstantAction { drive.useApril = true },
                drive.CorrectionAction(middleYellowPixel4, MecanumDrive.PARAMS.maxStackCorrectTimeSec.s),
                box.ejectTwoPixels(),
                InstantAction { drive.useApril = false },
                drive.actionBuilder(middleYellowPixel4)
                        .afterTime(Lift.LiftConfig.postBoardDecendWaitSec.s, systemsToIntake())
                        .setTangent(180.deg)
                        .splineToConstantHeading(park.position, 90.deg)
                        .build()
        )

        val actionMiddle = SequentialAction(
                drive.actionBuilder(startPose)
                        .strafeToLinearHeading(middlePurplePixel.position, middlePurplePixel.heading)
                        .stopAndAdd(intake.ejectPurple())
                        .afterTime(0.s, systemsToYellow())
                        .strafeTo(middleYellowPixel2.position)
                        .stopAndAdd(SequentialAction(
                                InstantAction { drive.useApril = true },
                                lift.goToLowYellow(),
                                box.ejectTwoPixels(),
                                lift.goToAboveWhite(),
                                InstantAction { drive.useApril = false }
                        ))
                        .afterTime(Lift.LiftConfig.postBoardDecendWaitSec.s, systemsToIntake())
                        .setTangent(middleBoardLeavingAngle)
                        .splineToConstantHeading(middlePostBoardRun2.position, 180.deg, speed60)
                        .splineToConstantHeading(middlePreStackRun2.position, 180.deg)
                        .afterTime(0.s, firstStackPrep())
                        .splineToConstantHeading(middleStacky2.position, 180.deg, speed60)
                        .build(),
                drive.CorrectionAction(middleStacky2, MecanumDrive.PARAMS.maxStackCorrectTimeSec.s),
                InstantAction { intake.boardSideFirstStack() },
                intake.waitForPixel(),
                InstantAction { box.power = 0.0 },
                drive.actionBuilder(middleStacky2)
                        .afterTime(0.s, intake.ejectPixels())
                        .setTangent(0.deg)
                        .splineToConstantHeading(middlePostStackRun2.position, 0.deg, speed60)
                        .afterTime(Lift.LiftConfig.boardSidePostStackRiseWaitSec.s, systemsToAboveWhite())
                        .splineToConstantHeading(middlePreBoardRun2.position, 0.deg)
                        .splineToConstantHeading(leftPreYellowPixel2.position, leftBoardAproachAngle, speed60)
                        .build(),
                InstantAction { drive.useApril = true },
                drive.CorrectionAction(leftYellowPixel3, MecanumDrive.PARAMS.maxStackCorrectTimeSec.s),
                box.ejectTwoPixels(),
                InstantAction { drive.useApril = false },
                drive.actionBuilder(leftYellowPixel3)
                        .afterTime(Lift.LiftConfig.postBoardDecendWaitSec.s, systemsToIntake())
                        .setTangent(leftBoardLeavingAngle)
                        .splineToConstantHeading(middlePostBoardRun3.position, 180.deg, speed60)
                        .splineToConstantHeading(middlePreStackRun3.position, 180.deg)
                        .afterTime(0.s, secondStackPrep())
                        .splineToConstantHeading(middleStacky3.position, 180.deg)
                        .build(),
                drive.CorrectionAction(middleStacky3, MecanumDrive.PARAMS.maxStackCorrectTimeSec.s),
                InstantAction { intake.boardSideSecondStack() },
                intake.waitForPixel(),
                InstantAction { box.power = 0.0 },
                drive.actionBuilder(middleStacky3)
                        .afterTime(0.s, intake.ejectPixels())
                        .setTangent(0.deg)
                        .splineToConstantHeading(middlePostStackRun3.position, 0.deg, speed60)
                        .afterTime(Lift.LiftConfig.boardSidePostStackRiseWaitSec.s, systemsToAboveWhite())
                        .splineToConstantHeading(middlePreBoardRun3.position, 0.deg)
                        .splineToConstantHeading(leftPreYellowPixel3.position, leftBoardAproachAngle, speed60)
                        .build(),
                InstantAction { drive.useApril = true },
                drive.CorrectionAction(leftYellowPixel4, MecanumDrive.PARAMS.maxStackCorrectTimeSec.s),
                box.ejectTwoPixels(),
                InstantAction { drive.useApril = false },
                drive.actionBuilder(leftYellowPixel4)
                        .afterTime(Lift.LiftConfig.postBoardDecendWaitSec.s, systemsToIntake())
                        .setTangent(180.deg)
                        .splineToConstantHeading(park.position, 90.deg)
                        .build()
        )

        val actionRight = SequentialAction(
                drive.actionBuilder(startPose)
                        .setTangent(-45.deg)
                        .splineToLinearHeading(rightPurplePixel, 180.deg)
                        .stopAndAdd(intake.ejectPurple())
                        .afterTime(0.s, systemsToYellow())
                        .strafeTo(rightYellowPixel2.position)
                        .stopAndAdd(SequentialAction(
                                InstantAction { drive.useApril = true },
                                lift.goToLowYellow(),
                                box.ejectTwoPixels(),
                                lift.goToAboveWhite(),
                                InstantAction { drive.useApril = false }
                        ))
                        .afterTime(Lift.LiftConfig.postBoardDecendWaitSec.s, systemsToIntake())
                        .setTangent(rightBoardLeavingAngle)
                        .splineToConstantHeading(rightPostBoardRun2.position, 180.deg, speed60)
                        .splineToConstantHeading(rightPreStackRun2.position, 180.deg)
                        .afterTime(0.s, firstStackPrep())
                        .splineToConstantHeading(rightStacky2.position, 180.deg, speed60)
                        .build(),
                drive.CorrectionAction(rightStacky2, MecanumDrive.PARAMS.maxStackCorrectTimeSec.s),
                InstantAction { intake.boardSideFirstStack() },
                intake.waitForPixel(),
                InstantAction { box.power = 0.0 },
                drive.actionBuilder(rightStacky2)
                        .afterTime(0.s, intake.ejectPixels())
                        .setTangent(0.deg)
                        .splineToConstantHeading(rightPostStackRun2.position, 0.deg, speed60)
                        .afterTime(Lift.LiftConfig.boardSidePostStackRiseWaitSec.s, systemsToAboveWhite())
                        .splineToConstantHeading(rightPreBoardRun2.position, 0.deg)
                        .splineToConstantHeading(leftPreYellowPixel2.position, leftBoardAproachAngle, speed60)
                        .build(),
                InstantAction { drive.useApril = true },
                drive.CorrectionAction(leftYellowPixel3, MecanumDrive.PARAMS.maxStackCorrectTimeSec.s),
                box.ejectTwoPixels(),
                InstantAction { drive.useApril = false },
                drive.actionBuilder(leftYellowPixel3)
                        .afterTime(Lift.LiftConfig.postBoardDecendWaitSec.s, systemsToIntake())
                        .setTangent(leftBoardLeavingAngle)
                        .splineToConstantHeading(rightPostBoardRun3.position, 180.deg, speed60)
                        .splineToConstantHeading(rightPreStackRun3.position, 180.deg)
                        .afterTime(0.s, secondStackPrep())
                        .splineToConstantHeading(rightStacky3.position, 180.deg)
                        .build(),
                drive.CorrectionAction(rightStacky3, MecanumDrive.PARAMS.maxStackCorrectTimeSec.s),
                InstantAction { intake.boardSideSecondStack() },
                intake.waitForPixel(),
                InstantAction { box.power = 0.0 },
                drive.actionBuilder(rightStacky3)
                        .afterTime(0.s, intake.ejectPixels())
                        .setTangent(0.deg)
                        .splineToConstantHeading(rightPostStackRun3.position, 0.deg, speed60)
                        .afterTime(Lift.LiftConfig.boardSidePostStackRiseWaitSec.s, systemsToAboveWhite())
                        .splineToConstantHeading(rightPreBoardRun3.position, 0.deg)
                        .splineToConstantHeading(leftPreYellowPixel3.position, leftBoardAproachAngle, speed60)
                        .build(),
                InstantAction { drive.useApril = true },
                drive.CorrectionAction(leftYellowPixel4, MecanumDrive.PARAMS.maxStackCorrectTimeSec.s),
                box.ejectTwoPixels(),
                InstantAction { drive.useApril = false },
                drive.actionBuilder(leftYellowPixel4)
                        .afterTime(Lift.LiftConfig.postBoardDecendWaitSec.s, systemsToIntake())
                        .setTangent(180.deg)
                        .splineToConstantHeading(park.position, 90.deg)
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

        while (isStarted && !isStopRequested && running) {
            val now = Time.now()
            mainDeltaTime = now - previousTime
            previousTime = now

            controlHub.clearBulkCache()

            val packet = TelemetryPacket()
            packet.fieldOverlay().operations.addAll(canvas.operations)

            running = action.run(packet)

            dash.sendTelemetryPacket(packet)

            telemetry.addData("main delta fps", min(1.s / mainDeltaTime, 200.0))
            telemetry.addData("main delta time ms", mainDeltaTime.ms)
            telemetry.addData("side delta fps", 1.s / sideDeltaTime)
            telemetry.addData("side delta time ms", sideDeltaTime.ms)
            telemetry.addData("lift pos", lift.positionTicks)
            telemetry.addData("lift target pos", lift.targetPositionTicks)
            telemetry.update()
        }
    }

    /*private fun firstStackPrep() = InstantFunction {
        intake.aboveFirstStack()
        intake.stackPower()
        box.power = 1.0
    }

    private fun secondStackPrep() = InstantFunction {
        intake.aboveSecondStack()
        intake.stackPower()
        box.power = 1.0
    }*/

    private fun firstStackPrep() = InstantFunction {
        intake.boardSideAboveFirstStack()
        intake.stackPower()
        box.power = 1.0
    }

    private fun secondStackPrep() = InstantFunction {
        intake.boardSideAboveSecondStack()
        intake.stackPower()
        box.power = 1.0
    }

    private fun thirdStackPrep() = InstantFunction {
        intake.aboveThirdStack()
        intake.stackPower()
        box.power = 1.0
    }

    private fun systemsToYellow() = SequentialAction(
        lift.goToPass(),
        ParallelAction(
            arm.scorePosQuick(),
            box.scorePosQuick()
        ),
        lift.goToYellow()
    )

    private fun systemsToAboveWhite() = SequentialAction(
        lift.goToPass(),
        ParallelAction(
            arm.scorePosQuick(),
            box.scorePosQuick()
        ),
        lift.goToAboveWhite()
    )

    private fun systemsToUpUp() = SequentialAction(
            lift.goToPass(),
            ParallelAction(
                    arm.scorePosQuick(),
                    box.scorePosQuick()
            ),
            lift.goToUpUp()
    )

    private fun systemsToIntake() = SequentialAction(
        lift.goToPass(),
        ParallelAction(
            arm.intakePosQuick(),
            box.intakePosQuick()
        ),
        InstantAction { box.power = 0.0 },
        lift.goToIntake(),
    )
}