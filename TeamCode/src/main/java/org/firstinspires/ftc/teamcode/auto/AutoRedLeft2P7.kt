package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
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
import org.firstinspires.ftc.teamcode.lib.action.DependendAction
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

@Config
@Photon
@Autonomous(preselectTeleOp = "LammaDriveRed")
class AutoRedLeft2P7 : MultiThreadOpMode() {
    companion object {
        @JvmField var startPose =             Pose(-1.5.tile, -61.inch, -90.deg)

        @JvmField var middleCycleOffset = 2.5.inch.y
        @JvmField var rightCycleOffset = 2.5.inch.y
        @JvmField var leftCycleOffset = 2.5.inch.y

        @JvmField var secondStackOffset = 2.inch.y

        @JvmField var middlePurplePixel =     Pose(-36 .inch, -16.inch, -60.deg)
        @JvmField var rightPurplePixel =      Pose(-32.inch, -35.inch, 0.deg)
        @JvmField var leftPurplePixel =       Pose(-47.5.inch, -16.inch, -90.deg)

        @JvmField var middleStacky1 =         Pose(-59.9.inch, -0.5.tile, 180.deg)
        @JvmField var rightStacky1 =          Pose(-59.9.inch, -0.5.tile, 180.deg)
        @JvmField var leftStacky1 =           Pose(-59.9.inch, -0.5.tile, 180.deg)

//        @JvmField var middlePreStacky1 =      middleStacky1 + 10.cm.x
//        @JvmField var rightPreStacky1 =       rightStacky1 + 10.cm.x
//        @JvmField var leftPreStacky1 =        leftStacky1 + 10.cm.x

        @JvmField var middlePostStackRun1 =   Pose(-30.inch, -0.5.tile, 180.deg)
        @JvmField var rightPostStackRun1 =    Pose(-30.inch, -0.5.tile, 180.deg)
        @JvmField var leftPostStackRun1 =     Pose(-30.inch, -0.5.tile, 180.deg)

        @JvmField var middlePreBoardRun1 =    Pose(18.inch, -0.5.tile, 180.deg)
        @JvmField var rightPreBoardRun1 =     Pose(18.inch, -0.5.tile, 180.deg)
        @JvmField var leftPreBoardRun1 =      Pose(18.inch, -0.5.tile, 180.deg)

        @JvmField var middleYellowPixel2 =    Pose(52.5.inch, -1.5.tile, 180.deg)
        @JvmField var rightYellowPixel2 =     Pose(52.5.inch, -43.5.inch, 180.deg)
        @JvmField var leftYellowPixel2 =      Pose(52.5.inch, -29.5.inch, 180.deg)

        @JvmField var middlePreYellowPixel1 = middleYellowPixel2
        //@JvmField var rightPreYellowPixel1 =  rightYellowPixel2
        @JvmField var leftPreYellowPixel1 =   leftYellowPixel2 - 2.6.inch.y

        @JvmField var middlePostBoardRun2 =   Pose(18.inch, -0.5.tile, 180.deg) + middleCycleOffset - 1.inch.y
        @JvmField var rightPostBoardRun2 =    Pose(18.inch, -0.5.tile, 180.deg) + rightCycleOffset - 1.inch.y
        @JvmField var leftPostBoardRun2 =     Pose(18.inch, -0.5.tile, 180.deg) + leftCycleOffset - 1.inch.y

        @JvmField var middlePreStackRun2 =    Pose(-30.inch, -0.5.tile, 180.deg) + middleCycleOffset - 1.inch.y
        @JvmField var rightPreStackRun2 =     Pose(-30.inch, -0.5.tile, 180.deg) + rightCycleOffset - 1.inch.y
        @JvmField var leftPreStackRun2 =      Pose(-30.inch, -0.5.tile, 180.deg) + leftCycleOffset - 1.inch.y

        @JvmField var middleStacky2 =         Pose(-58.inch, -0.5.tile, 180.deg) + middleCycleOffset - 5.2.inch.y
        @JvmField var rightStacky2 =          Pose(-58.inch, -0.5.tile, 180.deg) + rightCycleOffset - 5.2.inch.y
        @JvmField var leftStacky2 =           Pose(-58.inch, -0.5.tile, 180.deg) + leftCycleOffset - 5.2.inch.y

        @JvmField var middlePostStackRun2 =   Pose(-30.inch, -0.5.tile, 180.deg) + middleCycleOffset - 1.inch.y
        @JvmField var rightPostStackRun2 =    Pose(-30.inch, -0.5.tile, 180.deg) + rightCycleOffset - 1.inch.y
        @JvmField var leftPostStackRun2 =     Pose(-30.inch, -0.5.tile, 180.deg) + leftCycleOffset - 1.inch.y

        @JvmField var middlePreBoardRun2 =    Pose(18.inch, -0.5.tile, 180.deg) + middleCycleOffset - 2.inch.y
        @JvmField var rightPreBoardRun2 =     Pose(18.inch, -0.5.tile, 180.deg) + rightCycleOffset - 2.inch.y
        @JvmField var leftPreBoardRun2 =      Pose(18.inch, -0.5.tile, 180.deg) + leftCycleOffset - 2.inch.y

        @JvmField var middleYellowPixel3 =    middleYellowPixel2 + middleCycleOffset + 1.inch.x
        //@JvmField var rightYellowPixel3 =     rightYellowPixel2
        @JvmField var leftYellowPixel3 =      leftYellowPixel2 + leftCycleOffset + 1.inch.x

        @JvmField var middlePreYellowPixel2 = middleYellowPixel3
        //@JvmField var rightPreYellowPixel2 =  rightYellowPixel3
        @JvmField var leftPreYellowPixel2 =   leftYellowPixel3 - 6.5.inch.y

        @JvmField var middlePostBoardRun3 =   Pose(18.inch, -0.5.tile, 180.deg) + middleCycleOffset - 1.inch.y
        @JvmField var rightPostBoardRun3 =    Pose(18.inch, -0.5.tile, 180.deg) + rightCycleOffset - 1.inch.y
        @JvmField var leftPostBoardRun3 =     Pose(18.inch, -0.5.tile, 180.deg) + leftCycleOffset - 1.inch.y

        @JvmField var middlePreStackRun3 =    Pose(-30.inch, -0.5.tile, 180.deg) + middleCycleOffset - 1.inch.y
        @JvmField var rightPreStackRun3 =     Pose(-30.inch, -0.5.tile, 180.deg) + rightCycleOffset - 1.inch.y
        @JvmField var leftPreStackRun3 =      Pose(-30.inch, -0.5.tile, 180.deg) + leftCycleOffset - 1.inch.y

        @JvmField var middleStacky3 =         Pose(-58.inch, -0.5.tile, 180.deg) + middleCycleOffset - 1.inch.x - 5.2.inch.y
        @JvmField var rightStacky3 =          Pose(-58.inch, -0.5.tile, 180.deg) + rightCycleOffset - 1.inch.x - 5.2.inch.y
        @JvmField var leftStacky3 =           Pose(-58.inch, -0.5.tile, 180.deg) + leftCycleOffset - 1.inch.x - 5.2.inch.y

        @JvmField var middlePostStackRun3 =   Pose(-30.inch, -0.5.tile, 180.deg) + middleCycleOffset - 1.inch.y
        @JvmField var rightPostStackRun3 =    Pose(-30.inch, -0.5.tile, 180.deg) + rightCycleOffset - 1.inch.y
        @JvmField var leftPostStackRun3 =     Pose(-30.inch, -0.5.tile, 180.deg) + leftCycleOffset - 1.inch.y

        @JvmField var middlePreBoardRun3 =    Pose(18.inch, -0.5.tile, 180.deg) + middleCycleOffset - 2.inch.y
        @JvmField var rightPreBoardRun3 =     Pose(18.inch, -0.5.tile, 180.deg) + rightCycleOffset - 2.inch.y
        @JvmField var leftPreBoardRun3 =      Pose(18.inch, -0.5.tile, 180.deg) + leftCycleOffset - 2.inch.y

        @JvmField var middleYellowPixel4 =    middleYellowPixel3
        //@JvmField var rightYellowPixel4 =     rightYellowPixel3
        @JvmField var leftYellowPixel4 =      leftYellowPixel3

        @JvmField var middlePreYellowPixel3 = middleYellowPixel4
        //@JvmField var rightPreYellowPixel3 =  rightYellowPixel4
        @JvmField var leftPreYellowPixel3 =   leftYellowPixel4 - 6.5.inch.y

        @JvmField var middlePostBoardRun4 =   Pose(18.inch, -0.5.tile, 180.deg) + middleCycleOffset - 3.inch.y
        @JvmField var rightPostBoardRun4 =    Pose(18.inch, -0.5.tile, 180.deg) + rightCycleOffset - 3.inch.y
        @JvmField var leftPostBoardRun4 =     Pose(18.inch, -0.5.tile, 180.deg) + leftCycleOffset - 3.inch.y

        @JvmField var middlePreStackRun4 =    Pose(-30.inch, -0.5.tile, 180.deg) + middleCycleOffset - 3.inch.y
        @JvmField var rightPreStackRun4 =     Pose(-30.inch, -0.5.tile, 180.deg) + rightCycleOffset - 3.inch.y
        @JvmField var leftPreStackRun4 =      Pose(-30.inch, -0.5.tile, 180.deg) + leftCycleOffset - 3.inch.y

        @JvmField var middleStacky4 =         Pose(-55.inch, -1.tile, 180.deg) + middleCycleOffset - 8.inch.y
        @JvmField var rightStacky4 =          Pose(-55.3.inch, -1.tile, 180.deg) + rightCycleOffset - 8.inch.y
        @JvmField var leftStacky4 =           Pose(-55.3.inch, -1.tile, 180.deg) + leftCycleOffset - 8.inch.y

        @JvmField var middlePostStackRun4 =   Pose(-30.inch, -0.5.tile, 180.deg) + middleCycleOffset + secondStackOffset - 3.inch.y
        @JvmField var rightPostStackRun4 =    Pose(-30.inch, -0.5.tile, 180.deg) + rightCycleOffset + secondStackOffset - 3.inch.y
        @JvmField var leftPostStackRun4 =     Pose(-30.inch, -0.5.tile, 180.deg) + leftCycleOffset + secondStackOffset - 3.inch.y

        @JvmField var middlePreBoardRun4 =    Pose(18.inch, -0.5.tile, 180.deg) + middleCycleOffset + secondStackOffset - 4.inch.y
        @JvmField var rightPreBoardRun4 =     Pose(18.inch, -0.5.tile, 180.deg) + rightCycleOffset + secondStackOffset - 4.inch.y
        @JvmField var leftPreBoardRun4 =      Pose(18.inch, -0.5.tile, 180.deg) + leftCycleOffset + secondStackOffset - 4.inch.y

        @JvmField var middleYellowPixel5 =    middleYellowPixel4 + secondStackOffset
        //@JvmField var rightYellowPixel5 =     rightYellowPixel4
        @JvmField var leftYellowPixel5 =      leftYellowPixel4 + secondStackOffset

        @JvmField var middlePreYellowPixel4 = middleYellowPixel5
        //@JvmField var rightPreYellowPixel4 =  rightYellowPixel5
        @JvmField var leftPreYellowPixel4 =   leftYellowPixel5 + 1.5.inch.x - 6.5.inch.y

        @JvmField var middleBoardAproachAngle = -30.deg
        //@JvmField var rightBoardAproachAngle = -30.deg
        @JvmField var leftBoardAproachAngle = -30.deg

        @JvmField var middleBoardLeavingAngle = 150.deg
        @JvmField var rightBoardLeavingAngle = 150.deg
        @JvmField var leftBoardLeavingAngle = 150.deg

        @JvmField var middleSecondStackLeavingAngle = 30.deg
        @JvmField var rightSecondStackLeavingAngle = 30.deg
        @JvmField var leftSecondStackLeavingAngle = 30.deg
    }

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
        camera.setColor(ColorVisionProcessor.DetectionColor.RED)

        val speed60 = MinVelConstraint(listOf(
                drive.kinematics.WheelVelConstraint(60.0),
                AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel)
        ))

        val actionLeft = drive.actionBuilder(startPose)
            .strafeToLinearHeading(leftPurplePixel.position, leftPurplePixel.heading)
            .stopAndAdd(intake.ejectPurple())
            .afterTime(0.s, firstStackPrep())
            .setTangent(90.deg)
            .splineToLinearHeading(leftStacky1, 180.deg)
            .stopAndAdd(SequentialAction(
                InstantAction { intake.firstStack() },
                intake.waitForPixel(),
                InstantAction { box.power = 0.0 }
            ))
            .afterTime(0.s, intake.ejectPixels())
            .setTangent(0.deg)
            .splineToConstantHeading(leftPostStackRun1.position, 0.deg)
            .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToYellow())
            .splineToConstantHeading(leftPreBoardRun1.position, 0.deg)
            .splineToConstantHeading(middlePreYellowPixel1.position, middleBoardAproachAngle, speed60)
            .stopAndAdd(DependendAction(SequentialAction(
                InstantAction { box.power = -1.0 },
                color.waitTillYellow(),
                InstantAction {
                    box.power = 0.0
                    drive.updatePoseWithApril()
                }
            )) {
                drive.updatePoseEstimate()
            })
            .strafeTo(leftYellowPixel2.position)
            .stopAndAdd(DependendAction(SequentialAction(
                box.ejectYellowPixel(),
                InstantAction {
                    lift.targetPositionTicks = Lift.LiftConfig.aboveWhiteTicks
                    drive.updatePoseWithApril()
                }
            )) {
                drive.updatePoseEstimate()
            })
            .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToIntake())
            .setTangent(leftBoardLeavingAngle)
            .splineToConstantHeading(leftPostBoardRun2.position, 180.deg, speed60)
            .splineToConstantHeading(leftPreStackRun2.position, 180.deg)
            .afterTime(0.s, secondStackPrep())
            .splineToConstantHeading(leftStacky2.position, 180.deg)
            .stopAndAdd(SequentialAction(
                InstantAction { intake.secondStack() },
                intake.waitForPixel(),
                InstantAction { box.power = 0.0 }
            ))
            .afterTime(0.s, intake.ejectPixels())
            .setTangent(0.deg)
            .splineToConstantHeading(leftPostStackRun2.position, 0.deg)
            .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToAboveWhite())
            .splineToConstantHeading(leftPreBoardRun2.position, 0.deg)
            .splineToConstantHeading(middlePreYellowPixel2.position, middleBoardAproachAngle, speed60)
            .stopAndAdd(DependendAction(SequentialAction(
                box.ejectTwoPixels(),
                InstantAction { drive.updatePoseWithApril() }
            )) {
                drive.updatePoseEstimate()
            })
            .afterTime(Lift.LiftConfig.postBoardDecendWaitSec.s, systemsToIntake())
            .setTangent(middleBoardLeavingAngle)
            .splineToConstantHeading(leftPostBoardRun3.position, 180.deg, speed60)
            .splineToConstantHeading(leftPreStackRun3.position, 180.deg)
            .afterTime(0.s, thirdStackPrep())
            .splineToConstantHeading(leftStacky3.position, 180.deg)
            .stopAndAdd(SequentialAction(
                InstantAction { intake.thirdStack() },
                intake.waitForPixel(),
                InstantAction { box.power = 0.0 }
            ))
            .afterTime(0.s, intake.ejectPixels())
            .setTangent(0.deg)
            .splineToConstantHeading(leftPostStackRun3.position, 0.deg)
            .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToAboveWhite())
            .splineToConstantHeading(leftPreBoardRun3.position, 0.deg)
            .splineToConstantHeading(middlePreYellowPixel3.position, middleBoardAproachAngle, speed60)
            .stopAndAdd(DependendAction(SequentialAction(
                box.ejectTwoPixels(),
                InstantAction { drive.updatePoseWithApril() }
            )) {
                drive.updatePoseEstimate()
            })
            .afterTime(Lift.LiftConfig.postBoardDecendWaitSec.s, systemsToIntake())
            .setTangent(middleBoardLeavingAngle)
            .splineToConstantHeading(leftPostBoardRun4.position, 180.deg, speed60)
            .splineToConstantHeading(leftPreStackRun4.position, 180.deg)
            .afterTime(0.s, forthStackPrep())
            .splineToConstantHeading(leftStacky4.position, 180.deg)
            .stopAndAdd(SequentialAction(
                InstantAction { intake.forthStack() },
                intake.waitForPixel(),
                InstantAction { box.power = 0.0 }
            ))
            .afterTime(0.s, intake.ejectPixels())
            .setTangent(leftSecondStackLeavingAngle)
            .splineToConstantHeading(leftPostStackRun4.position, 0.deg)
            .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToUpUp())
            .splineToConstantHeading(leftPreBoardRun4.position, 0.deg)
            .splineToConstantHeading(middlePreYellowPixel4.position, middleBoardAproachAngle, speed60)
            .stopAndAdd(box.ejectTwoPixels())
            .build()

        val actionMiddle = drive.actionBuilder(startPose)
            .strafeToLinearHeading(middlePurplePixel.position, middlePurplePixel.heading)
            .stopAndAdd(intake.ejectPurple())
            .afterTime(0.s, firstStackPrep())
            .setTangent(90.deg)
            .splineToLinearHeading(middleStacky1, 180.deg)
            .stopAndAdd(SequentialAction(
                InstantAction { intake.firstStack() },
                intake.waitForPixel(),
                InstantAction { box.power = 0.0 }
            ))
            .afterTime(0.3.s, intake.ejectPixels())
            .setTangent(0.deg)
            .splineToConstantHeading(middlePostStackRun1.position, 0.deg)
            .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToYellow())
            .splineToConstantHeading(middlePreBoardRun1.position, 0.deg)
            .splineToConstantHeading(leftPreYellowPixel1.position, leftBoardAproachAngle, speed60)
            .stopAndAdd(DependendAction(SequentialAction(
                InstantAction { box.power = -1.0 },
                color.waitTillYellow(),
                InstantAction {
                    box.power = 0.0
                    drive.updatePoseWithApril()
                }
            )) {
                drive.updatePoseEstimate()
            })
            .strafeTo(middleYellowPixel2.position)
            .stopAndAdd(DependendAction(SequentialAction(
                box.ejectYellowPixel(),
                InstantAction {
                    lift.targetPositionTicks = Lift.LiftConfig.aboveWhiteTicks
                    //drive.updatePoseWithApril()
                }
            )) {
                drive.updatePoseEstimate()
            })
            .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToIntake())
            .setTangent(middleBoardLeavingAngle)
            .splineToConstantHeading(middlePostBoardRun2.position, 180.deg, speed60)
            .splineToConstantHeading(middlePreStackRun2.position, 180.deg)
            .afterTime(0.s, secondStackPrep())
            .splineToConstantHeading(middleStacky2.position, 180.deg)
            .stopAndAdd(SequentialAction(
                InstantAction { intake.secondStack() },
                intake.waitForPixel(),
                InstantAction { box.power = 0.0 }
            ))
            .afterTime(0.3.s, intake.ejectPixels())
            .setTangent(0.deg)
            .splineToConstantHeading(middlePostStackRun2.position, 0.deg)
            .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToAboveWhite())
            .splineToConstantHeading(middlePreBoardRun2.position, 0.deg)
            .splineToConstantHeading(leftPreYellowPixel2.position, leftBoardAproachAngle, speed60)
            .stopAndAdd(DependendAction(SequentialAction(
                box.ejectTwoPixels(),
                InstantAction { drive.updatePoseWithApril() }
            )) {
                drive.updatePoseEstimate()
            })
            .afterTime(Lift.LiftConfig.postBoardDecendWaitSec.s, systemsToIntake())
            .setTangent(leftBoardLeavingAngle)
            .splineToConstantHeading(middlePostBoardRun3.position, 180.deg, speed60)
            .splineToConstantHeading(middlePreStackRun3.position, 180.deg)
            .afterTime(0.s, thirdStackPrep())
            .splineToConstantHeading(middleStacky3.position, 180.deg)
            .stopAndAdd(SequentialAction(
                InstantAction { intake.thirdStack() },
                intake.waitForPixel(),
                InstantAction { box.power = 0.0 }
            ))
            .afterTime(0.3.s, intake.ejectPixels())
            .setTangent(0.deg)
            .splineToConstantHeading(middlePostStackRun3.position, 0.deg)
            .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToAboveWhite())
            .splineToConstantHeading(middlePreBoardRun3.position, 0.deg)
            .splineToConstantHeading(leftPreYellowPixel3.position, leftBoardAproachAngle, speed60)
            .stopAndAdd(DependendAction(SequentialAction(
                box.ejectTwoPixels(),
                InstantAction { drive.updatePoseWithApril() }
            )) {
                drive.updatePoseEstimate()
            })
            .afterTime(Lift.LiftConfig.postBoardDecendWaitSec.s, systemsToIntake())
            .setTangent(leftBoardLeavingAngle)
            .splineToConstantHeading(middlePostBoardRun4.position, 180.deg, speed60)
            .splineToConstantHeading(middlePreStackRun4.position, 180.deg)
            .afterTime(0.s, forthStackPrep())
            .splineToConstantHeading(middleStacky4.position, 180.deg)
            .stopAndAdd(SequentialAction(
                InstantAction { intake.forthStack() },
                intake.waitForPixel(),
                InstantAction { box.power = 0.0 }
            ))
            .afterTime(0.3.s, intake.ejectPixels())
            .setTangent(middleSecondStackLeavingAngle)
            .splineToConstantHeading(middlePostStackRun4.position, 0.deg)
            .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToUpUp())
            .splineToConstantHeading(middlePreBoardRun4.position, 0.deg)
            .splineToConstantHeading(leftPreYellowPixel4.position, leftBoardAproachAngle, speed60)
            .stopAndAdd(box.ejectTwoPixels())
            .build()

        val actionRight = drive.actionBuilder(startPose)
            .setTangent(135.deg)
            .splineToLinearHeading(rightPurplePixel, 0.deg)
            .stopAndAdd(intake.ejectPurple())
            .afterTime(0.s, firstStackPrep())
            .setTangent(180.deg)
            .splineTo(rightPurplePixel.position - 4.cm.x, 180.deg)
            .splineToLinearHeading(rightStacky1, 180.deg)
            .stopAndAdd(SequentialAction(
                InstantAction { intake.firstStack() },
                intake.waitForPixel(),
                InstantAction { box.power = 0.0 }
            ))
            .afterTime(0.s, intake.ejectPixels())
            .setTangent(0.deg)
            .splineToConstantHeading(rightPostStackRun1.position, 0.deg)
            .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToYellow())
            .splineToConstantHeading(rightPreBoardRun1.position, 0.deg)
            .splineToConstantHeading(leftPreYellowPixel1.position, leftBoardAproachAngle, speed60)
            .stopAndAdd(DependendAction(SequentialAction(
                InstantAction { box.power = -1.0 },
                color.waitTillYellow(),
                InstantAction {
                    box.power = 0.0
                    drive.updatePoseWithApril()
                }
            )) {
                drive.updatePoseEstimate()
            })
            .strafeTo(rightYellowPixel2.position)
            .stopAndAdd(DependendAction(SequentialAction(
                box.ejectYellowPixel(),
                InstantAction {
                    lift.targetPositionTicks = Lift.LiftConfig.aboveWhiteTicks
                    //drive.updatePoseWithApril()
                }
            )) {
                drive.updatePoseEstimate()
            })
            .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToIntake())
            .setTangent(rightBoardLeavingAngle)
            .splineToConstantHeading(rightPostBoardRun2.position, 180.deg, speed60)
            .splineToConstantHeading(rightPreStackRun2.position, 180.deg)
            .afterTime(0.s, forthStackPrep())
            .splineToConstantHeading(rightStacky2.position, 180.deg)
            .stopAndAdd(SequentialAction(
                InstantAction { intake.secondStack() },
                intake.waitForPixel(),
                InstantAction { box.power = 0.0 }
            ))
            .afterTime(0.s, intake.ejectPixels())
            .setTangent(0.deg)
            .splineToConstantHeading(rightPostStackRun2.position, 0.deg)
            .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToAboveWhite())
            .splineToConstantHeading(rightPreBoardRun2.position, 0.deg)
            .splineToConstantHeading(leftPreYellowPixel2.position, leftBoardAproachAngle, speed60)
            .stopAndAdd(DependendAction(SequentialAction(
                box.ejectTwoPixels(),
                InstantAction { drive.updatePoseWithApril() }
            )) {
                drive.updatePoseEstimate()
            })
            .afterTime(Lift.LiftConfig.postBoardDecendWaitSec.s, systemsToIntake())
            .setTangent(leftBoardLeavingAngle)
            .splineToConstantHeading(rightPostBoardRun3.position, 180.deg, speed60)
            .splineToConstantHeading(rightPreStackRun3.position, 180.deg)
            .afterTime(0.s, thirdStackPrep())
            .splineToConstantHeading(rightStacky3.position, 180.deg)
            .stopAndAdd(SequentialAction(
                InstantAction { intake.thirdStack() },
                intake.waitForPixel(),
                InstantAction { box.power = 0.0 }
            ))
            .afterTime(0.s, intake.ejectPixels())
            .setTangent(0.deg)
            .splineToConstantHeading(rightPostStackRun3.position, 0.deg)
            .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToAboveWhite())
            .splineToConstantHeading(rightPreBoardRun3.position, 0.deg)
            .splineToConstantHeading(leftPreYellowPixel3.position, leftBoardAproachAngle, speed60)
            .stopAndAdd(DependendAction(SequentialAction(
                box.ejectTwoPixels(),
                InstantAction { drive.updatePoseWithApril() }
            )) {
                drive.updatePoseEstimate()
            })
            .afterTime(Lift.LiftConfig.postBoardDecendWaitSec.s, systemsToIntake())
            .setTangent(leftBoardLeavingAngle)
            .splineToConstantHeading(rightPostBoardRun4.position, 180.deg, speed60)
            .splineToConstantHeading(rightPreStackRun4.position, 180.deg)
            .afterTime(0.s, forthStackPrep())
            .splineToConstantHeading(rightStacky4.position, 180.deg)
            .stopAndAdd(SequentialAction(
                InstantAction { intake.thirdStack() },
                intake.waitForPixel(),
                InstantAction { box.power = 0.0 }
            ))
            .afterTime(0.3.s, intake.ejectPixels())
            .setTangent(rightSecondStackLeavingAngle)
            .splineToConstantHeading(rightPostStackRun4.position, 0.deg)
            .afterTime(Lift.LiftConfig.postStackRiseWaitSec.s, systemsToUpUp())
            .splineToConstantHeading(rightPreBoardRun4.position, 0.deg)
            .splineToConstantHeading(leftPreYellowPixel4.position, leftBoardAproachAngle, speed60)
            .stopAndAdd(box.ejectTwoPixels())
            .build()

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
        var startTime = Time.now()

        while (isStarted && !isStopRequested && running) {
            val now = Time.now()
            mainDeltaTime = now - previousTime
            previousTime = now

            controlHub.clearBulkCache()

            val packet = TelemetryPacket()
            packet.fieldOverlay().operations.addAll(canvas.operations)

            running = action.run(packet)

            if (Time.now() - startTime >= 32.5.s) {
                running = false
            }

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

    private fun thirdStackPrep() = InstantFunction {
        intake.aboveThirdStack()
        intake.stackPower()
        box.power = 1.0
    }

    private fun forthStackPrep() = InstantFunction {
        intake.aboveForthStack()
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