package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.lib.hardware.controlHub
import org.firstinspires.ftc.teamcode.lib.hardware.expansionHub
import org.firstinspires.ftc.teamcode.lib.opmode.MultiThreadOpMode
import org.firstinspires.ftc.teamcode.lib.units.Pose
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.deg
import org.firstinspires.ftc.teamcode.lib.units.inch
import org.firstinspires.ftc.teamcode.lib.units.ms
import org.firstinspires.ftc.teamcode.lib.units.s
import org.firstinspires.ftc.teamcode.lib.vision.ColorVisionProcessor
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.systems.Camera
import org.firstinspires.ftc.teamcode.systems.Lift
import org.firstinspires.ftc.teamcode.systems.multi.ArmMulti.Companion.armMulti
import org.firstinspires.ftc.teamcode.systems.multi.BoxMulti.Companion.boxMulti
import org.firstinspires.ftc.teamcode.systems.multi.IntakeMulti.Companion.intakeMulti
import org.firstinspires.ftc.teamcode.systems.multi.LiftMulti.Companion.liftMulti
import kotlin.math.min

@Photon
@Autonomous(preselectTeleOp = "LammaDriveRed")
class AutoRedRightPreload : MultiThreadOpMode() {
    private val startPose =             Pose(12.inch, -61.inch, -90.deg)

    private val middlePurplePixel =     Pose(20.inch, -24.inch, 180.deg)
    private val rightPurplePixel =      Pose(31.inch, -32.inch, 180.deg)
    private val leftPurplePixel =       Pose(8.inch, -32.inch, 180.deg)

    private val park = Pose(46.inch, -60.inch, 180.deg)

    private val middleYellowPixel2 =    Pose(51.inch, -35.inch, 180.deg)
    private val rightYellowPixel2 =     Pose(52.inch, -40.inch, 180.deg)
    private val leftYellowPixel2 =      Pose(51.inch, -29.inch, 180.deg)

    private var mainThread: Thread? = null

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

        while (opModeInInit()) {
            camera.update()
        }

        while (isStarted && !isStopRequested && mainThread?.isInterrupted != true) {
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
        var mainDeltaTime: Time

        mainThread = Thread.currentThread()

        val controlHub = hardwareMap.controlHub()

        controlHub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL

        val dash = FtcDashboard.getInstance()

        telemetry = MultipleTelemetry(telemetry, dash.telemetry)

        drive.camera = camera
        camera.telemetry = telemetry
        camera.setColor(ColorVisionProcessor.DetectionColor.RED)

        val actionLeft = SequentialAction(
                drive.actionBuilder(startPose)
                        .setTangent(45.deg)
                        .splineToLinearHeading(leftPurplePixel, 180.deg)
                        .stopAndAdd(intake.ejectPurple())
                        .afterTime(0.s, systemsToYellow())
                        .strafeTo(leftYellowPixel2.position)
                        .stopAndAdd(SequentialAction(
                            lift.goToPos(Lift.LiftConfig.subYellowTicks),
                            drive.CorrectionAction(leftYellowPixel2, 1.s),
                            box.ejectTwoPixels(),
                            lift.goToAboveWhite()
                        ))
                        .setTangent(180.deg)
                        .lineToX(48.inch)
                        .strafeTo(park.position)
                        .afterTime(0.s, systemsToIntake())
                        .build(),
        )

        val actionMiddle = SequentialAction(
                drive.actionBuilder(startPose)
                        .strafeToLinearHeading(middlePurplePixel.position, middlePurplePixel.heading)
                        .stopAndAdd(intake.ejectPurple())
                        .afterTime(0.s, systemsToYellow())
                        .strafeToLinearHeading(middleYellowPixel2.position, middleYellowPixel2.heading)
                        .stopAndAdd(SequentialAction(
                            lift.goToPos(Lift.LiftConfig.subYellowTicks),
                            drive.CorrectionAction(middleYellowPixel2, 1.s),
                            box.ejectTwoPixels(),
                            lift.goToAboveWhite()
                        ))
                        .setTangent(180.deg)
                        .lineToX(48.inch)
                        .strafeTo(park.position)
                        .afterTime(0.s, systemsToIntake())
                        .build(),
        )

        val actionRight = SequentialAction(
                drive.actionBuilder(startPose)
                        .strafeToLinearHeading(rightPurplePixel.position, rightPurplePixel.heading)
                        .stopAndAdd(intake.ejectPurple())
                        .afterTime(0.s, systemsToYellow())
                        .strafeToLinearHeading(rightYellowPixel2.position, rightYellowPixel2.heading)
                        .stopAndAdd(SequentialAction(
                            drive.CorrectionAction(rightYellowPixel2, 1.s),
                            lift.goToPos(Lift.LiftConfig.subYellowTicks),
                            box.ejectTwoPixels(),
                            lift.goToAboveWhite(),
                        ))
                        .setTangent(180.deg)
                        .lineToX(48.inch)
                        .strafeTo(park.position)
                        .afterTime(0.s, systemsToIntake())
                        .build(),
        )

        while (opModeInInit()) {
            camera.displayDetection()
            telemetry.update()
            sleep(10)
        }

        val action = when(camera.detectionPosition) {
            ColorVisionProcessor.DetectionPosition.LEFT -> actionLeft
            ColorVisionProcessor.DetectionPosition.CENTER -> actionMiddle
            ColorVisionProcessor.DetectionPosition.RIGHT -> actionRight
        }

        camera.disableColorDetection()
        camera.stopStream()

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
    private fun systemsToYellow() = SequentialAction(
        lift.goToPass(),
        ParallelAction(
            arm.scorePosQuick(),
            box.scorePosQuick()
        ),
        lift.goToYellow()
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