package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.MinVelConstraint
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
import org.firstinspires.ftc.teamcode.systems.multi.ArmMulti.Companion.armMulti
import org.firstinspires.ftc.teamcode.systems.multi.BoxMulti.Companion.boxMulti
import org.firstinspires.ftc.teamcode.systems.multi.Color2Multi.Companion.color2Multi
import org.firstinspires.ftc.teamcode.systems.multi.IntakeMulti.Companion.intakeMulti
import org.firstinspires.ftc.teamcode.systems.multi.LiftMulti.Companion.liftMulti
import kotlin.math.min

@Photon
@Autonomous(preselectTeleOp = "LammaDriveRed", group = "CRI")
class CRIRedLeft : MultiThreadOpMode() {
    
    private val drive by opModeLazy {
        MecanumDrive(hardwareMap, Pose(0.cm, 0.cm, 0.deg).pose2d)
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
//            arm.write()
//            arm.update()
//            box.write()
//            box.update()
//            lift.read()
//            lift.write()
//            lift.update()
//            intake.write()
//            intake.update()
//            color.read()
        }
    }

    override fun mainRunOpMode() {
        val oldTileSize = tile
        //tile = 61.cm

        val stackWait = 0.4.s
        val boardWait = 0.7.s
        val purpleWait = 0.4.s

        val startPose = Pose(-2.5.tile, -2.5.tile - 1.inch, -90.deg)

        val midPurplePixel = Pose(-2.5.tile, -0.5.tile - 4.inch, -90.deg)
        val leftPurplePixel = Pose(-3.tile, -1.tile, -90.deg)
        val rightPurplePixel = Pose(-2.tile, -1.tile, -90.deg)

        val midStacky = Pose(-3.5.tile, -0.5.tile, 180.deg)
        val leftStacky = midStacky
        val rightStacky = midStacky

        val midTransition = Pose(2.tile, -0.5.tile, 180.deg)
        val leftTransition = midTransition
        val rightTransition = midTransition

        val midCenterBoard = Pose(3.tile + 3.inch, -1.5.tile, 180.deg)
        val leftLeftBoard = Pose(3.tile + 3.inch, -1.5.tile + 6.inch, 180.deg)
        val rightRightBoard = Pose(3.tile + 3.inch, -1.5.tile - 6.inch, 180.deg)

        val midBoardAproachAngle = -30.deg
        val leftBoardAproachAngle = midBoardAproachAngle
        val rightBoardAproachAngle = midBoardAproachAngle

        val midBoardLeavingAngle = 150.deg
        val leftBoardLeavingAngle = midBoardLeavingAngle
        val rightBoardLeavingAngle = midBoardLeavingAngle

        val speed60 = MinVelConstraint(listOf(
            drive.kinematics.WheelVelConstraint(60.0),
            AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel)
        ))

        drive.pose = startPose.pose2d
        drive.imuStartHeading = drive.pose.heading.toDouble()

        tile = oldTileSize

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
                .strafeToLinearHeading(leftPurplePixel.position, leftPurplePixel.heading)
                .waitSeconds(purpleWait)
                .setTangent(90.deg)
                .splineToLinearHeading(leftStacky, 180.deg)
                .waitSeconds(stackWait)
                .setTangent(0.deg)
                .splineToConstantHeading(leftTransition.position, 0.deg)
                .splineToConstantHeading(leftLeftBoard.position, leftBoardAproachAngle, speed60)
                .waitSeconds(boardWait)
                .setTangent(leftBoardLeavingAngle)
                .splineToConstantHeading(leftTransition.position, 180.deg)
                .splineToConstantHeading(leftStacky.position, 180.deg)
                .waitSeconds(stackWait)
                .setTangent(0.deg)
                .splineToConstantHeading(leftTransition.position, 0.deg)
                .splineToConstantHeading(leftLeftBoard.position, leftBoardAproachAngle, speed60)
                .build()
        )

        val actionMiddle = SequentialAction(
            drive.actionBuilder(startPose)
                .strafeToLinearHeading(midPurplePixel.position, midPurplePixel.heading)
                .waitSeconds(purpleWait)
                .setTangent(90.deg)
                .splineToLinearHeading(midStacky, 180.deg)
                .waitSeconds(stackWait)
                .setTangent(0.deg)
                .splineToConstantHeading(midTransition.position, 0.deg)
                .splineToConstantHeading(midCenterBoard.position, midBoardAproachAngle, speed60)
                .waitSeconds(boardWait)
                .setTangent(midBoardLeavingAngle)
                .splineToConstantHeading(midTransition.position, 180.deg)
                .splineToConstantHeading(midStacky.position, 180.deg)
                .waitSeconds(stackWait)
                .setTangent(0.deg)
                .splineToConstantHeading(midTransition.position, 0.deg)
                .splineToConstantHeading(midCenterBoard.position, midBoardAproachAngle, speed60)
                .build()
        )

        val actionRight = SequentialAction(
            drive.actionBuilder(startPose)
                .strafeToLinearHeading(rightPurplePixel.position, rightPurplePixel.heading)
                .waitSeconds(purpleWait)
                .setTangent(90.deg)
                .splineToLinearHeading(rightStacky, 180.deg)
                .waitSeconds(stackWait)
                .setTangent(0.deg)
                .splineToConstantHeading(rightTransition.position, 0.deg)
                .splineToConstantHeading(rightRightBoard.position, rightBoardAproachAngle, speed60)
                .waitSeconds(boardWait)
                .setTangent(rightBoardLeavingAngle)
                .splineToConstantHeading(rightTransition.position, 180.deg)
                .splineToConstantHeading(rightStacky.position, 180.deg)
                .waitSeconds(stackWait)
                .setTangent(0.deg)
                .splineToConstantHeading(rightTransition.position, 0.deg)
                .splineToConstantHeading(rightRightBoard.position, rightBoardAproachAngle, speed60)
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
}