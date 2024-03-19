package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.lib.hardware.controlHub
import org.firstinspires.ftc.teamcode.lib.hardware.expansionHub
import org.firstinspires.ftc.teamcode.lib.opmode.MultiThreadOpMode
import org.firstinspires.ftc.teamcode.lib.units.Pose
import org.firstinspires.ftc.teamcode.lib.units.SleepAction
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.cm
import org.firstinspires.ftc.teamcode.lib.units.deg
import org.firstinspires.ftc.teamcode.lib.units.inch
import org.firstinspires.ftc.teamcode.lib.units.ms
import org.firstinspires.ftc.teamcode.lib.units.s
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.systems.Camera
import org.firstinspires.ftc.teamcode.systems.Intake
import org.firstinspires.ftc.teamcode.systems.multi.ArmMulti.Companion.armMulti
import org.firstinspires.ftc.teamcode.systems.multi.BoxMulti.Companion.boxMulti
import org.firstinspires.ftc.teamcode.systems.multi.IntakeMulti.Companion.intakeMulti
import org.firstinspires.ftc.teamcode.systems.multi.LiftMulti.Companion.liftMulti
import kotlin.math.min

@Photon
@Autonomous(preselectTeleOp = "LammaDrive")
class AutoRedLeft : MultiThreadOpMode() {
    private val startPose = Pose(-36.inch, -61.inch, -90.deg)

    private val middlePurplePixel = Pose(-36.inch, -33.inch, -90.deg)

    private val rightPurplePixel = Pose(-34.inch + 8.cm, -45.inch + 12.cm, -135.deg)
    private val rightPrePurplePixel = Pose(-36.inch, -47.inch, -90.deg)

    private val leftPurplePixel = Pose(-47.inch, -38.inch, -90.deg)

    private val middleYellowPixel = Pose(47.inch, -36.inch - 2.cm, 180.deg)

    private val leftYellowPixel = Pose(47.inch, -30.inch + 2.cm, 180.deg)

    private val rightYellowPixel = Pose(47.inch + 1.cm, -42.inch, 180.deg)

    private val middleRun1 = Pose(24.inch, -12.inch + 1.cm, 180.deg)
    private val middleRun2 = Pose(-30.inch, -12.inch + 1.cm, 180.deg)
    private val preStacky = Pose(-57.inch, -50.inch, -180.deg)
    private val stacky = Pose (-54.inch - 9.cm + 10.cm, -12.inch - 16.cm + 16.cm, 180.deg)
    private val stacky2 = stacky - 12.cm.x
    private val stacky3 = stacky2

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

    private var sideDeltaTime = 20.ms

    override fun sideRunOpMode() {
        var previousTime = Time.now()

        val expansionHub = hardwareMap.expansionHub()

        expansionHub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL

        drive.camera = camera

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
        }
    }

    override fun mainRunOpMode() {
        var previousTime = Time.now()
        var deltaTime: Time

        val controlHub = hardwareMap.controlHub()

        controlHub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL

        val dash = FtcDashboard.getInstance()

        telemetry = MultipleTelemetry(telemetry, dash.telemetry)

        val slowSpeed = MinVelConstraint(
            listOf(
                drive.kinematics.WheelVelConstraint(10.0),
                AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel)
            )
        )

        val actionMiddle = SequentialAction(
            drive.actionBuilder(startPose)
                .strafeToLinearHeading(middlePurplePixel.position, middlePurplePixel.heading)
                .setTangent(-90.deg)
                .splineTo(preStacky.position, 180.deg)
                .strafeTo(stacky.position)
                .stopAndAdd(InstantAction {
                    intake.aboveStack()
                    intake.stackPower()
                })
                .strafeTo(stacky2.position)
                .build(),
            /*ParallelAction(
                drive.actionBuilder(stacky2)
                    .strafeTo(stacky3.position, slowSpeed)
                    .build(),
                takePixelsIntake()
            ),
            InstantAction { intake.power = 0.0 },
            drive.actionBuilder(stacky3)
                .setTangent(0.deg)
                .afterTime(0.s, lift.goToTicks(Lift.autoRampTicks))
                .afterTime(1.s, ejectPixels())
                .splineTo(middleRun2.position, 0.deg)
                .splineTo(middleRun1.position, 0.deg)
                .afterTime(0.s, SequentialAction(
                    lift.goToPass(),
                    ParallelAction(
                        claw.clawToScore(),
                        arm.goToScore(),
                    ),
                    lift.goToTicks(Lift.aboveYellowTicks)
                ))
                .splineTo(middleYellowPixel.position - 10.cm.x, 0.deg)
                .stopAndAdd(resetPose())
                .strafeToLinearHeading(middleYellowPixel.position, middleYellowPixel.heading)
                .stopAndAdd(SequentialAction(
                    SleepAction(0.2.s),
                    ParallelAction(
                        claw.openLeft(),
                        claw.openRight()
                    ),
                    SleepAction(0.2.s),
                    lift.goToPass(),
                ))
                .strafeTo(middleYellowPixel.position - 6.inch.x)
                .afterTime(0.s, SequentialAction(
                    claw.closeClaw(),
                    ParallelAction(
                        claw.clawToRamp(),
                        arm.goToRamp()
                    ),
                    SleepAction(1.0.s),
                    lift.goToRamp()
                ))
                .strafeTo(middleYellowPixel.position + 22.inch.y - 6.inch.x)
                .build()*/
            InstantAction {
                intake.stackPower()
                box.power = 1.0
                intake.firstStack()
            },
            SleepAction(Intake.IntakeConfig.intakeWaitSec.s),
            InstantAction {
                intake.power = 0.0
                box.power = 0.0
                          },
            drive.actionBuilder(stacky3)
                .setTangent(0.deg)
                .afterTime(1.s, intake.ejectPixels())
                .splineToConstantHeading(middleRun2.position, 0.deg)
                .afterTime(0.5.s, lift.goToYellow())
                .afterTime(0.5.s, ParallelAction(
                    box.goToScore(),
                    arm.goToScore()
                ))
                .splineToConstantHeading(middleRun1.position, 0.deg)
                .splineToConstantHeading(middleYellowPixel.position, -30.deg)
                /*.afterTime(0.s, SequentialAction(
                    lift.goToPass(),
                    ParallelAction(
                        claw.clawToScore(),
                        arm.goToScore(),
                    ),
                    lift.goToTicks(Lift.yellowPixelTicks)
                ))
                .splineTo(middleYellowPixel.position - 10.cm.x, 0.deg)
                .stopAndAdd(resetPose())
                .strafeToLinearHeading(middleYellowPixel.position, middleYellowPixel.heading)
                .stopAndAdd(SequentialAction(
                    SleepAction(0.2.s),
                    ParallelAction(
                        claw.openLeft(),
                        claw.openRight()
                    ),
                    SleepAction(0.2.s),
                    lift.goToPass(),
                ))
                .afterTime(0.s, SequentialAction(
                    claw.closeClaw(),
                    ParallelAction(
                        claw.clawToRamp(),
                        arm.goToRamp()
                    ),
                    lift.goToRamp()
                ))
                .setTangent(180.deg)
                .splineTo(middleRun1.position, 180.deg)
                .splineTo(middleRun2.position, 180.deg)
                .afterTime(0.s, ParallelAction(
                    InstantAction {
                        intake.position = Intake.IntakeConfig.hitStack
                        intake.power = 0.8
                    },
                    claw.openRamp()
                )
                )
                .splineTo(stacky2.position, 180.deg)
                .stopAndAdd(SequentialAction(
                    InstantAction { intake.position = 1.0 },
                    SleepAction(0.2.s),
                ))*/
                .build(),
            /*ParallelAction(
                drive.actionBuilder(stacky2)
                    .strafeTo(stacky3.position, slowSpeed)
                    .build(),
                takePixelsIntake()
            ),
            InstantAction { intake.power = 0.0 },
            drive.actionBuilder(stacky3)
                .setTangent(0.deg)
                .afterTime(0.s, lift.goToTicks(Lift.autoRampTicks))
                .afterTime(1.s, ejectPixels())
                .splineTo(middleRun2.position, 0.deg)
                .splineTo(middleRun1.position, 0.deg)
                .afterTime(0.s, SequentialAction(
                    lift.goToPass(),
                    ParallelAction(
                        claw.clawToScore(),
                        arm.goToScore(),
                    ),
                    lift.goToTicks(Lift.aboveYellowTicks)
                ))
                .splineTo(middleYellowPixel.position - 10.cm.x, 0.deg)
                .stopAndAdd(resetPose())
                .strafeToLinearHeading(middleYellowPixel.position, middleYellowPixel.heading)
                .stopAndAdd(SequentialAction(
                    SleepAction(0.2.s),
                    ParallelAction(
                        claw.openLeft(),
                        claw.openRight()
                    ),
                    SleepAction(0.2.s),
                    lift.goToPass(),
                ))
                .strafeTo(middleYellowPixel.position - 6.inch.x)
                .afterTime(0.s, SequentialAction(
                    claw.closeClaw(),
                    ParallelAction(
                        claw.clawToRamp(),
                        arm.goToRamp()
                    ),
                    SleepAction(1.0.s),
                    lift.goToRamp()
                ))
                .strafeTo(middleYellowPixel.position + 22.inch.y - 6.inch.x)
                .build()*/
        )

        while (opModeInInit()) {
            sleep(10)
        }

        val action = actionMiddle

        val canvas = Canvas()
        action.preview(canvas)

        var running = true

        while (isStarted && !isStopRequested && running) {
            val now = Time.now()
            deltaTime = now - previousTime
            previousTime = now

            controlHub.clearBulkCache()

            val packet = TelemetryPacket()
            packet.fieldOverlay().operations.addAll(canvas.operations)

            running = action.run(packet)

            dash.sendTelemetryPacket(packet)

            telemetry.addData("main delta fps", min(1.s / deltaTime, 200.0))
            telemetry.addData("main delta time ms", deltaTime.ms)
            telemetry.addData("side delta fps", 1.s / sideDeltaTime)
            telemetry.addData("side delta time ms", sideDeltaTime.ms)
            telemetry.addData("lift pos", lift.positionTicks)
            telemetry.addData("lift target pos", lift.targetPositionTicks)
            telemetry.update()
        }
    }
}