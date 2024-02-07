package org.firstinspires.ftc.teamcode.regionala

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.lib.opmode.MultiThreadOpMode
import org.firstinspires.ftc.teamcode.lib.units.Pose
import org.firstinspires.ftc.teamcode.lib.units.SleepAction
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.deg
import org.firstinspires.ftc.teamcode.lib.units.inch
import org.firstinspires.ftc.teamcode.lib.units.ms
import org.firstinspires.ftc.teamcode.lib.units.s
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDriveEx
import org.firstinspires.ftc.teamcode.robot.ArmMulti.Companion.armMulti
import org.firstinspires.ftc.teamcode.robot.Camera
import org.firstinspires.ftc.teamcode.robot.ClawMulti.Companion.clawMulti
import org.firstinspires.ftc.teamcode.robot.ColorSensorsMulti.Companion.colorSensMulti
import org.firstinspires.ftc.teamcode.robot.ColorVisionProcessor
import org.firstinspires.ftc.teamcode.robot.IntakeMulti.Companion.intakeMulti
import org.firstinspires.ftc.teamcode.robot.LiftMulti.Companion.liftMulti
import org.firstinspires.ftc.teamcode.robot.hardware.controlHub
import org.firstinspires.ftc.teamcode.robot.hardware.expansionHub

@Autonomous
@Photon
class AutoRedRight : MultiThreadOpMode() {
    private val startPose = Pose(12.inch, -61.inch, -90.deg)
    private val middlePurplePixel = Pose(12.inch, -33.inch, -90.deg)
    private val middleYellowPixel = Pose(46.inch, -36.inch, 180.deg)
    private val middleRun1 = Pose(24.inch, -60.inch, 180.deg)

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
        var deltaTime: Time

        val controlHub = hardwareMap.controlHub()

        controlHub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL

        val dash = FtcDashboard.getInstance()

        telemetry = MultipleTelemetry(telemetry, dash.telemetry)

        camera.telemetry = telemetry
        camera.setColor(ColorVisionProcessor.DetectionColor.RED)

        val actionMiddle = SequentialAction(
            ParallelAction(
                drive.actionBuilder(startPose)
                    .strafeToLinearHeading(middlePurplePixel.position, middlePurplePixel.heading)
                    .setTangent(-90.deg)
                    .splineToLinearHeading(middleYellowPixel, 0.deg)
                    .build(),
                SequentialAction(
                    SleepAction(1.s),
                    lift.goToPass(),
                    ParallelAction(
                        claw.clawToScore(),
                        arm.goToScore(),
                    )
                )
            ),
            claw.openRight(),
            SleepAction(0.2.s),
            ParallelAction(
                drive.actionBuilder(middleYellowPixel)
                    .setTangent(180.deg)
                    .splineToConstantHeading(middleRun1.position, 180.deg)
                    .build(),
                SequentialAction(
                    SleepAction(0.2.s),
                    claw.closeClaw(),
                    ParallelAction(
                        claw.clawToRamp(),
                        arm.goToRamp()
                    ),
                    lift.goToRamp()
                )
            )
        )

        while (opModeInInit()) {
            camera.displayDetection()
            telemetry.update()
            sleep(10)
        }

        val action = when(camera.detectionPosition) {
            ColorVisionProcessor.DetectionPosition.LEFT -> actionMiddle
            ColorVisionProcessor.DetectionPosition.CENTER -> actionMiddle
            ColorVisionProcessor.DetectionPosition.RIGHT -> actionMiddle
        }

        val c = Canvas()
        action.preview(c)

        var running = true

        while (isStarted && !isStopRequested /*&& running*/) {
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

            telemetry.addData("main delta fps", 1.s / deltaTime)
            telemetry.addData("side delta time", 1.s / sideDeltaTime)
            telemetry.addData("running", running )
            telemetry.update()
        }
    }
}