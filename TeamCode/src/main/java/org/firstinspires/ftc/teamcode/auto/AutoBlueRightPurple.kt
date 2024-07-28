package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.lib.hardware.controlHub
import org.firstinspires.ftc.teamcode.lib.hardware.expansionHub
import org.firstinspires.ftc.teamcode.lib.opmode.MultiThreadOpMode
import org.firstinspires.ftc.teamcode.lib.units.Pose
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.cm
import org.firstinspires.ftc.teamcode.lib.units.deg
import org.firstinspires.ftc.teamcode.lib.units.inch
import org.firstinspires.ftc.teamcode.lib.units.ms
import org.firstinspires.ftc.teamcode.lib.units.s
import org.firstinspires.ftc.teamcode.lib.vision.ColorVisionProcessor
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.systems.Camera
import org.firstinspires.ftc.teamcode.systems.multi.IntakeMulti.Companion.intakeMulti
import kotlin.math.min

@Photon
@Autonomous(preselectTeleOp = "LammaDriveBlue")
class AutoBlueRightPurple : MultiThreadOpMode() {
    private val startPose =             Pose(-36.inch, 61.inch, 90.deg)

    private val middlePurplePixel =     Pose(-38.inch, 16.inch, 90.deg)
    private val leftPurplePixel =       Pose(-32.inch, 35.inch, 0.deg)
    private val rightPurplePixel =      Pose(-47.inch, 16.inch, 90.deg)

    private val drive by opModeLazy {
        MecanumDrive(hardwareMap, startPose.pose2d)
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

        while (isStarted && !isStopRequested) {
            val now = Time.now()
            sideDeltaTime = now - previousTime
            previousTime = now

            expansionHub.clearBulkCache()

            camera.update()
            intake.write()
            intake.update()
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

        val actionLeft = drive.actionBuilder(startPose)
            .setTangent(-135.deg)
            .splineToLinearHeading(leftPurplePixel, 0.deg)
            .stopAndAdd(intake.ejectPurple())
            .setTangent(180.deg)
            .lineToX(leftPurplePixel.position.x - 2.cm)
            .build()

        val actionMiddle = drive.actionBuilder(startPose)
            .strafeTo(middlePurplePixel.position)
            .stopAndAdd(intake.ejectPurple())
            .setTangent(-90.deg)
            .lineToY(middlePurplePixel.position.y - 2.cm)
            .build()

        val actionRight = drive.actionBuilder(startPose)
            .strafeTo(rightPurplePixel.position)
            .stopAndAdd(intake.ejectPurple())
            .setTangent(-90.deg)
            .lineToY(rightPurplePixel.position.y - 2.cm)
            .build()

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
            telemetry.update()
        }
    }
}