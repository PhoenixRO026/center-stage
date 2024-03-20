package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
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
import org.firstinspires.ftc.teamcode.systems.multi.ColorMulti.Companion.colorMulti
import org.firstinspires.ftc.teamcode.systems.multi.IntakeMulti.Companion.intakeMulti
import org.firstinspires.ftc.teamcode.systems.multi.LiftMulti.Companion.liftMulti
import kotlin.math.min

@Photon
@Autonomous(preselectTeleOp = "LammaDrive")
class AutoRedLeft : MultiThreadOpMode() {
    private val startPose = Pose(-36.inch, -61.inch, -90.deg)

    private val middlePurplePixel = Pose(-36.inch, -16.inch, -90.deg)

    private val rightPurplePixel = Pose(-30.inch, -36.inch, 0.deg)
    private val rightPrePurplePixel = Pose(-38.inch, -45.inch, 0.deg)

    private val leftPurplePixel = Pose(-47.inch, -38.inch, -90.deg)

    private val middleYellowPixel = Pose(49.5.inch, -36.inch - 2.cm, 180.deg)

    private val leftYellowPixel = Pose(49.5.inch, -32.inch, 180.deg)
    private val leftYellowPixel2 = Pose(49.8.inch, -30.inch, 180.deg)

    private val rightYellowPixel = Pose(49.5.inch, -42.inch, 180.deg)

    private val middleRun1 = Pose(18.inch, -12.inch + 1.cm, 180.deg)
    private val middleRun2 = Pose(-30.inch, -12.inch + 1.cm, 180.deg)
    private val preStacky = Pose(-55.inch, -50.inch, -180.deg)
    private val stacky = Pose (-54.inch - 9.cm + 10.cm, -12.inch, 180.deg)
    private val stacky2 = stacky - 12.cm.x
    private val stacky3 = stacky2 - 2.cm.x + 3.cm.y

    private val rightCaseOffset = 8.cm.x + 2.cm.y

    private val cycleOffset = 5.cm.y

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
        hardwareMap.colorMulti()
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
            color.read()
        }
    }

    override fun mainRunOpMode() {
        var previousTime = Time.now()
        var deltaTime: Time

        val controlHub = hardwareMap.controlHub()

        controlHub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL

        val dash = FtcDashboard.getInstance()

        telemetry = MultipleTelemetry(telemetry, dash.telemetry)

        val slow30Speed = MinVelConstraint(
            listOf(
                drive.kinematics.WheelVelConstraint(30.0),
                AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel)
            )
        )

        val slow50Speed = MinVelConstraint(
            listOf(
                drive.kinematics.WheelVelConstraint(50.0),
                AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel)
            )
        )

        val slowDecel = ProfileAccelConstraint(-15.0, 80.0)

        val actionRight = SequentialAction(
            drive.actionBuilder(startPose)
                .setTangent(135.deg)
                .splineToLinearHeading(rightPurplePixel, 0.deg)
                .stopAndAdd(intake.ejectPurple())
                .setTangent(180.deg)
                .afterTime(0.s, InstantAction {
                    intake.aboveStack()
                    intake.stackPower()
                    box.power = 1.0
                })
                .splineTo(rightPurplePixel.position - 4.cm.x, 180.deg)
                .splineToLinearHeading(stacky + rightCaseOffset, 180.deg)
                .strafeTo(stacky2.position + rightCaseOffset)
                .build(),
            drive.CorrectionAction(stacky2 + rightCaseOffset, 1.s),
            InstantAction {
                intake.firstStack()
            },
            color.waitForFrontPixel(),
            InstantAction { box.power = 0.0 },
            drive.actionBuilder(stacky2 + rightCaseOffset)
                .setTangent(0.deg)
                .afterTime(0.s, intake.ejectPixels())
                .splineToConstantHeading(middleRun2.position + rightCaseOffset, 0.deg)
                .afterTime(0.3.s, lift.goToPass())
                .afterTime(0.8.s, InstantAction {
                    arm.scorePosNow()
                    box.scorePosNow()
                })
                .afterTime(1.3.s, lift.goToYellow())
                .splineToConstantHeading(middleRun1.position + rightCaseOffset, 0.deg)
                .splineToConstantHeading(leftYellowPixel.position + rightCaseOffset, -30.deg, /*slow50Speed, slowDecel*/)
                .build(),
            InstantAction{ drive.useApril = true },
            drive.CorrectionAction(leftYellowPixel, 1.s),
            SleepAction(0.1.s),
            InstantAction {
                box.power = -1.0
            },
            SleepAction(0.25.s),
            InstantAction {
                box.power = 0.0
                drive.useApril = false
            },
            SleepAction(0.1.s),
            drive.actionBuilder(leftYellowPixel)
                .strafeTo(rightYellowPixel.position)
                .build(),
            drive.CorrectionAction(rightYellowPixel, 1.s),
            SleepAction(0.1.s),
            InstantAction {
                box.power = -1.0
            },
            SleepAction(0.45.s),
            drive.actionBuilder(rightYellowPixel)
                .afterTime(0.s, lift.goToPass())
                .afterTime(0.3.s, SequentialAction(
                    InstantAction {
                        box.intakePosNow()
                        arm.intakePosNow()
                    },
                    SleepAction(0.5.s),
                    lift.goToIntake(),
                    InstantAction {
                        box.power = 0.0
                    }
                ))
                .setTangent(150.deg)
                .splineToConstantHeading(middleRun1.position + cycleOffset, 180.deg)
                .splineToConstantHeading(middleRun2.position + cycleOffset, 180.deg)
                .splineToConstantHeading(stacky.position + cycleOffset, 180.deg)
                .afterTime(0.s, InstantAction {
                    intake.firstStack()
                    intake.stackPower()
                    box.power = 1.0
                })
                .strafeTo(stacky3.position + cycleOffset)
                .build(),
            drive.CorrectionAction(stacky3 + cycleOffset, 1.s),
            InstantAction {
                intake.secondStack()
            },
            color.waitForPixels(),
            InstantAction { box.power = 0.0 },
            drive.actionBuilder(stacky3 + cycleOffset)
                .setTangent(0.deg)
                .afterTime(0.s, intake.ejectPixels())
                .splineToConstantHeading(middleRun2.position + cycleOffset, 0.deg)
                .afterTime(0.3.s, lift.goToPass())
                .afterTime(0.8.s, InstantAction {
                    arm.scorePosNow()
                    box.scorePosNow()
                })
                //.afterTime(1.3.s, lift.goToYellow())
                .splineToConstantHeading(middleRun1.position + cycleOffset, 0.deg)
                .splineToConstantHeading(leftYellowPixel2.position + cycleOffset, -30.deg, /*slow50Speed, slowDecel*/)
                .build(),
            InstantAction{ drive.useApril = true },
            drive.CorrectionAction(leftYellowPixel2, 1.s),
            SleepAction(0.1.s),
            InstantAction {
                box.power = -1.0
            },
            SleepAction(0.6.s),
            InstantAction {
                drive.useApril = false
            },
            drive.actionBuilder(leftYellowPixel2)
                .afterTime(0.s, lift.goToPass())
                .afterTime(0.3.s, SequentialAction(
                    InstantAction {
                        box.intakePosNow()
                        arm.intakePosNow()
                    },
                    SleepAction(0.5.s),
                    lift.goToIntake(),
                    InstantAction {
                        box.power = 0.0
                    }
                ))
                .setTangent(150.deg)
                .splineToConstantHeading(middleRun1.position + cycleOffset, 180.deg)
                .splineToConstantHeading(middleRun2.position + cycleOffset, 180.deg)
                .splineToConstantHeading(stacky.position + cycleOffset, 180.deg)
                .afterTime(0.s, InstantAction {
                    intake.secondStack()
                    intake.stackPower()
                    box.power = 1.0
                })
                .strafeTo(stacky3.position + cycleOffset)
                .build(),
            drive.CorrectionAction(stacky3 + cycleOffset, 1.s),
            InstantAction {
                intake.thirdStack()
            },
            color.waitForPixels(),
            InstantAction { box.power = 0.0 },
            drive.actionBuilder(stacky3 + cycleOffset)
                .setTangent(0.deg)
                .afterTime(0.s, intake.ejectPixels())
                .splineToConstantHeading(middleRun2.position + cycleOffset, 0.deg)
                .afterTime(0.3.s, lift.goToPass())
                .afterTime(0.8.s, InstantAction {
                    arm.scorePosNow()
                    box.scorePosNow()
                })
                //.afterTime(1.3.s, lift.goToYellow())
                .splineToConstantHeading(middleRun1.position + cycleOffset, 0.deg)
                .splineToConstantHeading(leftYellowPixel2.position + cycleOffset, -30.deg, /*slow50Speed, slowDecel*/)
                .build(),
            InstantAction{ drive.useApril = true },
            drive.CorrectionAction(leftYellowPixel2, 1.s),
            SleepAction(0.1.s),
            InstantAction {
                box.power = -1.0
            },
            SleepAction(0.6.s),
            InstantAction {
                drive.useApril = false
            },
        )

        val actionMiddle = SequentialAction(
            drive.actionBuilder(startPose)
                .strafeToLinearHeading(middlePurplePixel.position, middlePurplePixel.heading)
                .stopAndAdd(intake.ejectPurple())
                .setTangent(90.deg)
                .afterTime(0.s, InstantAction {
                    intake.aboveStack()
                    intake.stackPower()
                    box.power = 1.0
                })
                .splineToLinearHeading(stacky, 180.deg)
                .strafeTo(stacky2.position)
                .build(),
            drive.CorrectionAction(stacky2, 1.s),
            InstantAction {
                intake.firstStack()
            },
            color.waitForFrontPixel(),
            InstantAction { box.power = 0.0 },
            drive.actionBuilder(stacky2)
                .setTangent(0.deg)
                .afterTime(0.s, intake.ejectPixels())
                .splineToConstantHeading(middleRun2.position, 0.deg)
                .afterTime(0.3.s, lift.goToPass())
                .afterTime(0.8.s, InstantAction {
                    arm.scorePosNow()
                    box.scorePosNow()
                })
                .afterTime(1.3.s, lift.goToYellow())
                .splineToConstantHeading(middleRun1.position, 0.deg)
                .splineToConstantHeading(leftYellowPixel.position, -30.deg, /*slow50Speed, slowDecel*/)
                .build(),
            InstantAction{ drive.useApril = true },
            drive.CorrectionAction(leftYellowPixel, 1.s),
            SleepAction(0.1.s),
            InstantAction {
                box.power = -1.0
            },
            SleepAction(0.25.s),
            InstantAction {
                box.power = 0.0
                drive.useApril = false
            },
            SleepAction(0.1.s),
            drive.actionBuilder(leftYellowPixel)
                .strafeTo(middleYellowPixel.position)
                .build(),
            drive.CorrectionAction(middleYellowPixel, 1.s),
            SleepAction(0.1.s),
            InstantAction {
                box.power = -1.0
            },
            SleepAction(0.45.s),
            drive.actionBuilder(middleYellowPixel)
                .afterTime(0.s, lift.goToPass())
                .afterTime(0.3.s, SequentialAction(
                    InstantAction {
                        box.intakePosNow()
                        arm.intakePosNow()
                    },
                    SleepAction(0.5.s),
                    lift.goToIntake(),
                    InstantAction {
                        box.power = 0.0
                    }
                ))
                .setTangent(150.deg)
                .splineToConstantHeading(middleRun1.position + cycleOffset, 180.deg)
                .splineToConstantHeading(middleRun2.position + cycleOffset, 180.deg)
                .splineToConstantHeading(stacky.position + cycleOffset, 180.deg)
                .afterTime(0.s, InstantAction {
                    intake.firstStack()
                    intake.stackPower()
                    box.power = 1.0
                })
                .strafeTo(stacky3.position + cycleOffset)
                .build(),
            drive.CorrectionAction(stacky3 + cycleOffset, 1.s),
            InstantAction {
                intake.secondStack()
            },
            color.waitForPixels(),
            InstantAction { box.power = 0.0 },
            drive.actionBuilder(stacky3 + cycleOffset)
                .setTangent(0.deg)
                .afterTime(0.s, intake.ejectPixels())
                .splineToConstantHeading(middleRun2.position + cycleOffset, 0.deg)
                .afterTime(0.3.s, lift.goToPass())
                .afterTime(0.8.s, InstantAction {
                    arm.scorePosNow()
                    box.scorePosNow()
                })
                //.afterTime(1.3.s, lift.goToYellow())
                .splineToConstantHeading(middleRun1.position + cycleOffset, 0.deg)
                .splineToConstantHeading(leftYellowPixel2.position + cycleOffset, -30.deg, /*slow50Speed, slowDecel*/)
                .build(),
            InstantAction{ drive.useApril = true },
            drive.CorrectionAction(leftYellowPixel2, 1.s),
            SleepAction(0.1.s),
            InstantAction {
                box.power = -1.0
            },
            SleepAction(0.6.s),
            InstantAction {
                drive.useApril = false
            },
            drive.actionBuilder(leftYellowPixel2)
                .afterTime(0.s, lift.goToPass())
                .afterTime(0.3.s, SequentialAction(
                    InstantAction {
                        box.intakePosNow()
                        arm.intakePosNow()
                    },
                    SleepAction(0.5.s),
                    lift.goToIntake(),
                    InstantAction {
                        box.power = 0.0
                    }
                ))
                .setTangent(150.deg)
                .splineToConstantHeading(middleRun1.position + cycleOffset, 180.deg)
                .splineToConstantHeading(middleRun2.position + cycleOffset, 180.deg)
                .splineToConstantHeading(stacky.position + cycleOffset, 180.deg)
                .afterTime(0.s, InstantAction {
                    intake.secondStack()
                    intake.stackPower()
                    box.power = 1.0
                })
                .strafeTo(stacky3.position + cycleOffset)
                .build(),
            drive.CorrectionAction(stacky3 + cycleOffset, 1.s),
            InstantAction {
                intake.thirdStack()
            },
            color.waitForPixels(),
            InstantAction { box.power = 0.0 },
            drive.actionBuilder(stacky3 + cycleOffset)
                .setTangent(0.deg)
                .afterTime(0.s, intake.ejectPixels())
                .splineToConstantHeading(middleRun2.position + cycleOffset, 0.deg)
                .afterTime(0.3.s, lift.goToPass())
                .afterTime(0.8.s, InstantAction {
                    arm.scorePosNow()
                    box.scorePosNow()
                })
                //.afterTime(1.3.s, lift.goToYellow())
                .splineToConstantHeading(middleRun1.position + cycleOffset, 0.deg)
                .splineToConstantHeading(leftYellowPixel2.position + cycleOffset, -30.deg, /*slow50Speed, slowDecel*/)
                .build(),
            InstantAction{ drive.useApril = true },
            drive.CorrectionAction(leftYellowPixel2, 1.s),
            SleepAction(0.1.s),
            InstantAction {
                box.power = -1.0
            },
            SleepAction(0.6.s),
            InstantAction {
                drive.useApril = false
            },
        )

        while (opModeInInit()) {
            sleep(10)
        }

        val action = actionRight

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