package org.firstinspires.ftc.teamcode.regionala

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.lib.opmode.MultiThreadOpMode
import org.firstinspires.ftc.teamcode.lib.units.Pose
import org.firstinspires.ftc.teamcode.lib.units.SleepAction
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.cm
import org.firstinspires.ftc.teamcode.lib.units.deg
import org.firstinspires.ftc.teamcode.lib.units.inch
import org.firstinspires.ftc.teamcode.lib.units.ms
import org.firstinspires.ftc.teamcode.lib.units.rad
import org.firstinspires.ftc.teamcode.lib.units.s
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDriveEx
import org.firstinspires.ftc.teamcode.robot.ArmMulti.Companion.armMulti
import org.firstinspires.ftc.teamcode.robot.Camera
import org.firstinspires.ftc.teamcode.robot.ClawMulti.Companion.clawMulti
import org.firstinspires.ftc.teamcode.robot.ColorSensorsMulti.Companion.colorSensMulti
import org.firstinspires.ftc.teamcode.robot.ColorVisionProcessor
import org.firstinspires.ftc.teamcode.robot.Intake
import org.firstinspires.ftc.teamcode.robot.IntakeMulti.Companion.intakeMulti
import org.firstinspires.ftc.teamcode.robot.Lift
import org.firstinspires.ftc.teamcode.robot.LiftMulti.Companion.liftMulti
import org.firstinspires.ftc.teamcode.robot.hardware.controlHub
import org.firstinspires.ftc.teamcode.robot.hardware.expansionHub
import kotlin.math.min

@Autonomous
@Photon
class AutoBlueLeft : MultiThreadOpMode() {
    private val startPose = Pose(12.inch, 61.inch, 90.deg)
    private val middlePurplePixel = Pose(12.inch, 33.inch, 90.deg)
    private val leftPurplePixel = Pose(24.inch, 45.inch, 90.deg)
    private val rightPurplePixel = Pose(10.inch - 10.cm, 45.inch - 10.cm, 45.deg)
    private val rightPrePurplePixel = Pose(12.inch, 49.inch, 90.deg)
    private val middleYellowPixel = Pose(47.inch, 36.inch - 2.cm, 180.deg)
    private val leftYellowPixel = Pose(47.inch, 42.inch - 2.cm, 180.deg)
    private val rightYellowPixel = Pose(47.inch, 30.inch - 2.cm, 180.deg)
    private val middleRun1 = Pose(24.inch, 60.inch - 4.cm, 180.deg)
    private val middleRun2 = Pose(-32.inch, 60.inch - 4.cm, 180.deg)
    private val middleRun3 = middleRun2 - 7.cm.y
    private val stacky = Pose (-54.inch - 14.cm, 36.inch + 16.cm, 180.deg)
    private val stacky2 = stacky - 23.cm.y
    private val stacky3 = stacky2 + 10.cm.x

    private val drive by opModeLazy {
        MecanumDriveEx(hardwareMap, startPose)
    }

    private val arm by opModeLazy {
        hardwareMap.armMulti()
    }

    private val claw by opModeLazy {
        hardwareMap.clawMulti()
    }

    private val lift by opModeLazy {
        hardwareMap.liftMulti()
    }

    private val colorSensors by opModeLazy {
        hardwareMap.colorSensMulti()
    }

    private val intake by opModeLazy {
        hardwareMap.intakeMulti()
    }
    private val camera by opModeLazy {
        Camera(hardwareMap, telemetry)
    }

    private var sideDeltaTime = 20.ms

    override fun sideRunOpMode() {
        var previousTime = System.currentTimeMillis().ms

        val expansionHub = hardwareMap.expansionHub()

        expansionHub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL

        waitForStart()

        while (isStarted && !isStopRequested) {
            val now = System.currentTimeMillis().ms
            sideDeltaTime = now - previousTime
            previousTime = now

            expansionHub.clearBulkCache()

            arm.write()
            arm.update(sideDeltaTime)
            claw.write()
            claw.update(sideDeltaTime)
            lift.read()
            lift.write()
            intake.write()
            intake.update(sideDeltaTime)
            colorSensors.read()
        }
    }

    override fun mainRunOpMode() {
        var previousTime = System.currentTimeMillis().ms
        var deltaTime: Time = 20.ms

        val controlHub = hardwareMap.controlHub()

        controlHub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL

        val slowSpeed = MinVelConstraint(
            listOf(
                drive.kinematics.WheelVelConstraint(10.0),
                AngularVelConstraint(MecanumDriveEx.PARAMS.maxAngVel)
            )
        )

        val dash = FtcDashboard.getInstance()

        telemetry = MultipleTelemetry(telemetry, dash.telemetry)

        camera.telemetry = telemetry
        camera.setColor(ColorVisionProcessor.DetectionColor.BLUE)

        val actionLeft = SequentialAction(
            ParallelAction(
                drive.actionBuilder(startPose)
                    .strafeToLinearHeading(leftPurplePixel.position, leftPurplePixel.heading)
                    .setTangent(90.deg)
                    .splineToLinearHeading(leftYellowPixel - 8.cm.x, 0.deg)
                    .stopAndAdd(resetPose())
                    .strafeToLinearHeading(leftYellowPixel.position, leftYellowPixel.heading)
                    .build(),
                SequentialAction(
                    SleepAction(1.s),
                    lift.goToPass(),
                    ParallelAction(
                        claw.clawToScore(),
                        arm.goToScore(),
                    ),
                    lift.goToTicks(Lift.yellowPixelTicks)
                )
            ),
            SleepAction(0.2.s),
            claw.openRight(),
            SleepAction(0.2.s),
            lift.goToPass(),
            ParallelAction(
                drive.actionBuilder(leftYellowPixel)
                    .setTangent(180.deg)
                    .splineTo(middleRun1.position, 180.deg)
                    .splineTo(middleRun2.position, 180.deg)
                    .afterTime(0.s, ParallelAction(
                        InstantAction { intake.position = Intake.IntakeConfig.hitStack },
                        claw.openRamp()
                    ))
                    .splineTo(stacky.position, 180.deg)
                    .strafeTo(stacky2.position)
                    .build(),
                SequentialAction(
                    claw.closeClaw(),
                    ParallelAction(
                        claw.clawToRamp(),
                        arm.goToRamp()
                    ),
                    lift.goToRamp()
                )
            ),
            InstantAction {
                intake.power = 1.0
                intake.position = 1.0
            },
            SleepAction(0.2.s),
            ParallelAction(
                drive.actionBuilder(stacky2)
                    .strafeTo(stacky3.position, slowSpeed)
                    .build(),
                takePixelsIntake()
            ),
            drive.actionBuilder(stacky3)
                .setTangent(0.deg)
                .afterTime(1.s, ejectPixels())
                .splineTo(middleRun3.position, 0.deg)
                .splineTo(middleRun1.position, 0.deg)
                .afterTime(0.s, SequentialAction(
                    lift.goToPass(),
                    ParallelAction(
                        claw.clawToScore(),
                        arm.goToScore(),
                    ),
                    lift.goToTicks(Lift.aboveYellowTicks)
                ))
                .splineTo(leftYellowPixel.position - 10.cm.x, 0.deg)
                .stopAndAdd(resetPose())
                .strafeToLinearHeading(leftYellowPixel.position, leftYellowPixel.heading)
                .build(),
            SleepAction(0.2.s),
            ParallelAction(
                claw.openLeft(),
                claw.openRight()
            ),
            SleepAction(0.2.s),
            lift.goToPass(),
            claw.closeClaw(),
            drive.actionBuilder(leftYellowPixel)
                .strafeTo(leftYellowPixel.position - 4.inch.x)
                .afterTime(0.s, ParallelAction(
                    claw.clawToRamp(),
                    arm.goToRamp()
                ),)
                .strafeTo(leftYellowPixel.position + 16.inch.y - 4.inch.x)
                .build(),
            SleepAction(0.5.s),
            lift.goToRamp()
        )

        val actionRight = SequentialAction(
            ParallelAction(
                drive.actionBuilder(startPose)
                    .setTangent(-90.deg)
                    .splineTo(rightPrePurplePixel.position, -90.deg)
                    .splineTo(rightPurplePixel.position, -135.deg)
                    //.strafeToLinearHeading(leftPurplePixel.position, leftPurplePixel.heading)
                    .setTangent(0.deg)
                    .splineToLinearHeading(rightYellowPixel - 8.cm.x, 0.deg)
                    .stopAndAdd(resetPose())
                    .strafeToLinearHeading(rightYellowPixel.position, rightYellowPixel.heading)
                    .build(),
                SequentialAction(
                    SleepAction(1.s),
                    lift.goToPass(),
                    ParallelAction(
                        claw.clawToScore(),
                        arm.goToScore(),
                    ),
                    lift.goToTicks(Lift.yellowPixelTicks)
                )
            ),
            SleepAction(0.2.s),
            claw.openRight(),
            SleepAction(0.2.s),
            lift.goToPass(),
            ParallelAction(
                drive.actionBuilder(rightYellowPixel)
                    .setTangent(180.deg)
                    .splineTo(middleRun1.position, 180.deg)
                    .splineTo(middleRun2.position, 180.deg)
                    .afterTime(0.s, ParallelAction(
                        InstantAction { intake.position = Intake.IntakeConfig.hitStack },
                        claw.openRamp()
                    ))
                    .splineTo(stacky.position, 180.deg)
                    .strafeTo(stacky2.position)
                    .build(),
                SequentialAction(
                    claw.closeClaw(),
                    ParallelAction(
                        claw.clawToRamp(),
                        arm.goToRamp()
                    ),
                    lift.goToRamp()
                )
            ),
            InstantAction {
                intake.power = 1.0
                intake.position = 1.0
            },
            SleepAction(0.2.s),
            ParallelAction(
                drive.actionBuilder(stacky2)
                    .strafeTo(stacky3.position, slowSpeed)
                    .build(),
                takePixelsIntake()
            ),
            drive.actionBuilder(stacky3)
                .setTangent(0.deg)
                .afterTime(1.s, ejectPixels())
                .splineTo(middleRun3.position, 0.deg)
                .splineTo(middleRun1.position, 0.deg)
                .afterTime(0.s, SequentialAction(
                    lift.goToPass(),
                    ParallelAction(
                        claw.clawToScore(),
                        arm.goToScore(),
                    ),
                    lift.goToTicks(Lift.aboveYellowTicks)
                ))
                .splineTo(rightYellowPixel.position - 10.cm.x, 0.deg)
                .stopAndAdd(resetPose())
                .strafeToLinearHeading(rightYellowPixel.position, rightYellowPixel.heading)
                .build(),
            SleepAction(0.2.s),
            ParallelAction(
                claw.openLeft(),
                claw.openRight()
            ),
            SleepAction(0.2.s),
            lift.goToPass(),
            claw.closeClaw(),
            drive.actionBuilder(rightYellowPixel)
                .strafeTo(rightYellowPixel.position - 4.inch.x)
                .afterTime(0.s, ParallelAction(
                    claw.clawToRamp(),
                    arm.goToRamp()
                ),)
                .strafeTo(rightYellowPixel.position + 28.inch.y - 4.inch.x)
                .build(),
            SleepAction(0.5.s),
            lift.goToRamp()
        )

        val actionMiddle = SequentialAction(
            ParallelAction(
                drive.actionBuilder(startPose)
                    .strafeToLinearHeading(middlePurplePixel.position, middlePurplePixel.heading)
                    .setTangent(90.deg)
                    .splineToLinearHeading(middleYellowPixel - 8.cm.x, 0.deg)
                    .stopAndAdd(resetPose())
                    .strafeToLinearHeading(middleYellowPixel.position, middleYellowPixel.heading)
                    .build(),
                SequentialAction(
                    SleepAction(1.s),
                    lift.goToPass(),
                    ParallelAction(
                        claw.clawToScore(),
                        arm.goToScore(),
                    ),
                    lift.goToTicks(Lift.yellowPixelTicks)
                )
            ),
            SleepAction(0.2.s),
            claw.openRight(),
            SleepAction(0.2.s),
            lift.goToPass(),
            ParallelAction(
                drive.actionBuilder(middleYellowPixel)
                    .setTangent(180.deg)
                    .splineTo(middleRun1.position, 180.deg)
                    .splineTo(middleRun2.position, 180.deg)
                    .afterTime(0.s, ParallelAction(
                        InstantAction { intake.position = Intake.IntakeConfig.hitStack },
                        claw.openRamp()
                    ))
                    .splineTo(stacky.position, 180.deg)
                    .strafeTo(stacky2.position)
                    .build(),
                SequentialAction(
                    claw.closeClaw(),
                    ParallelAction(
                        claw.clawToRamp(),
                        arm.goToRamp()
                    ),
                    lift.goToRamp()
                )
            ),
            InstantAction {
                intake.power = 1.0
                intake.position = 1.0
            },
            SleepAction(0.2.s),
            ParallelAction(
                drive.actionBuilder(stacky2)
                    .strafeTo(stacky3.position, slowSpeed)
                    .build(),
                takePixelsIntake()
            ),
            drive.actionBuilder(stacky3)
                .setTangent(0.deg)
                .afterTime(1.s, ejectPixels())
                .splineTo(middleRun3.position, 0.deg)
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
                .build(),
            SleepAction(0.2.s),
            ParallelAction(
                claw.openLeft(),
                claw.openRight()
            ),
            SleepAction(0.2.s),
            lift.goToPass(),
            claw.closeClaw(),
            drive.actionBuilder(middleYellowPixel)
                .strafeTo(middleYellowPixel.position + 22.inch.y - 4.inch.x)
                .build(),
            ParallelAction(
                claw.clawToRamp(),
                arm.goToRamp()
            ),
            SleepAction(0.5.s),
            lift.goToRamp()
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

        val c = Canvas()
        action.preview(c)

        var running = true

        while (isStarted && !isStopRequested && running) {
            val now = System.currentTimeMillis().ms
            deltaTime = now - previousTime
            previousTime = now

            controlHub.clearBulkCache()

            val p = TelemetryPacket()
            p.fieldOverlay().operations.addAll(c.operations)

            if (running) {
                running = action.run(p)
            }

            dash.sendTelemetryPacket(p)

            telemetry.addData("main delta fps", min(1.s / deltaTime, 200.0))
            telemetry.addData("side delta time", 1.s / sideDeltaTime)
            telemetry.addData("running", running )
            telemetry.addData("robot pose x inch", drive.pose.position.x)
            telemetry.addData("robot pose x inch", drive.pose.position.y)
            telemetry.addData("robot pose heading deg", drive.pose.heading.log().rad.deg)
            telemetry.addData("camera pose x inch", camera.robotPose.position.x)
            telemetry.addData("camera pose y inch", camera.robotPose.position.y)
            telemetry.addData("camera pose heading deg", camera.robotPose.heading.deg)
            telemetry.addData("imu heading deg", drive.imuHeading.rad.deg)
            telemetry.update()
        }
    }

    private fun resetPose() = object : Action {
        var init = true
        var startTime = 0.ms

        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false
                startTime = System.currentTimeMillis().ms
                camera.enableAprilTagDetection()
            }

            if ((System.currentTimeMillis().ms - startTime) > 2.s) return false

            return if (camera.findTag2()) {
                drive.pose = Pose2d(camera.robotPose.position.inch, drive.pose.heading)
                camera.disableAprilTagDetection()
                false
            } else {
                true
            }
        }

    }

    private fun takePixelsIntake() = SequentialAction(
        ParallelAction(
            SequentialAction(
                colorSensors.waitForRightPixel(),
                claw.closeRight()
            ),
            SequentialAction(
                colorSensors.waitForLeftPixel(),
                claw.closeLeft()
            )
        ),
        InstantAction {
            intake.position = 0.0
        }
    )

    private fun leftPixelIntake() = SequentialAction(
        InstantAction { intake.power = 1.0 },
        ParallelAction(
            intake.waitForPos(Intake.aboveStack),
            claw.openRamp()
        ),
        InstantAction { intake.targetPosition = Intake.IntakeConfig.stack1 },
        colorSensors.waitForLeftPixel(),
        InstantAction {
            intake.position = Intake.aboveStack
            claw.leftFingerPosition = 1.0
        }
    )

    private fun rightPixelIntake() = SequentialAction(
        intake.waitForPos(Intake.IntakeConfig.stack1),
        InstantAction { intake.targetPosition = Intake.IntakeConfig.stack2 },
        colorSensors.waitForRightPixel(),
        InstantAction {
            intake.position = 0.0
            intake.power = 0.0
            claw.rightFingerPosition = 1.0
        }
    )

    private fun ejectPixels() = SequentialAction(
        InstantAction { intake.power = -1.0 },
        SleepAction(1.0.s),
        InstantAction { intake.power = 0.0 }
    )
}