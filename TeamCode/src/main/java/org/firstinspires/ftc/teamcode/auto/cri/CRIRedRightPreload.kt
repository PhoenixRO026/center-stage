package org.firstinspires.ftc.teamcode.auto.cri

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.SequentialAction
import com.outoftheboxrobotics.photoncore.Photon
import com.phoenix.phoenixlib.units.Distance2d
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
import org.firstinspires.ftc.teamcode.systems.multi.ArmMulti.Companion.armMulti
import org.firstinspires.ftc.teamcode.systems.multi.BoxMulti.Companion.boxMulti
import org.firstinspires.ftc.teamcode.systems.multi.Color2Multi.Companion.color2Multi
import org.firstinspires.ftc.teamcode.systems.multi.IntakeMulti.Companion.intakeMulti
import org.firstinspires.ftc.teamcode.systems.multi.LiftMulti.Companion.liftMulti
import java.util.ArrayDeque

@Photon
@Autonomous(preselectTeleOp = "LammaDriveRed", group = "CRI")
class CRIRedRightPreload : MultiThreadOpMode() {

    private val avgWindow = 100

    private val startPose = Pose(1.5.tile, -2.5.tile - 2.inch, -90.deg)

    private val midPurplePixel = Pose(1.5.tile, -1.5.tile + 2.inch, 90.deg)
    private val leftPurplePixel = Pose(1.5.tile - 2.inch, -1.5.tile, 180.deg)
    private val rightPurplePixel = Pose(-0.5.tile + 2.inch, -1.5.tile, 0.deg)

    private val midTile = Distance2d(-0.5.tile, -1.5.tile)

    private val wallClearance = 10.cm

    private val clearance = 10.cm
    
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

        val actionLeft = SequentialAction(
            drive.actionBuilder(startPose)
                .strafeTo(midTile)
                .turnTo(leftPurplePixel.heading)
                .strafeTo(leftPurplePixel.position)
                .stopAndAdd(intake.ejectPurple())
                .strafeTo(leftPurplePixel.position + clearance.x)
                .build()
        )

        val actionMiddle = SequentialAction(
            drive.actionBuilder(startPose)
                .strafeTo(midTile)
                .turnTo(midPurplePixel.heading)
                .strafeTo(midPurplePixel.position)
                .stopAndAdd(intake.ejectPurple())
                .strafeTo(midPurplePixel.position - clearance.y)
                .build()
        )

        val actionRight = SequentialAction(
            drive.actionBuilder(startPose)
                .strafeTo(midTile)
                .turnTo(rightPurplePixel.heading)
                .strafeTo(rightPurplePixel.position)
                .stopAndAdd(intake.ejectPurple())
                .strafeTo(rightPurplePixel.position - clearance.x)
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
}