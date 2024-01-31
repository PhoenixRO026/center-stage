package org.firstinspires.ftc.teamcode.evenimente.regio

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.phoenix_ro026.phoenixlib.units.Pose
import com.phoenix_ro026.phoenixlib.units.cm
import com.phoenix_ro026.phoenixlib.units.deg
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.systems.DetectionPipeline
import org.firstinspires.ftc.teamcode.evenimente.regio.robot.Robot4

@Autonomous
class StackTestMovement : LinearOpMode() {

    @Config
    data object StackConfig {
        @JvmField var intakeAngle = 0.52
        @JvmField var intakePower = 1.0
    }
    override fun runOpMode() {
        val dash = FtcDashboard.getInstance()
        val startPose = Pose2d(0.0, 0.0, Math.toRadians(0.0))
        val robot = Robot4(hardwareMap, telemetry)

        val drive = robot.drive
        robot.camera.setColor(DetectionPipeline.DetectionColor.RED)
        robot.camera.openCamera()

        val actionLeft = SequentialAction(
                robot.claw.openClawRamp(),
                InstantAction { robot.intake.power = 1.0 },
                robot.intake.goToAngle(0.535),
                SleepAction(3.0),
                robot.intake.goToAngle(0.0),
                drive.actionBuilder(startPose)
                        .setTangent(Math.toRadians(-90.0))
                        .lineToY(4.5)
                        .build(),
                SleepAction(1.0),
                robot.intake.goToAngle(0.64),
                SleepAction(3.0),
                robot.intake.goToAngle(0.0),
                InstantAction { robot.intake.power = 0.0 },
                robot.claw.closeClaw()
        )
        while (opModeInInit()) {
            robot.camera.displayDetection()
            telemetry.update()
            sleep(20)
        }
        val action = when (robot.camera.detectionPosition) {
            DetectionPipeline.DetectionPosition.LEFT -> actionLeft
            DetectionPipeline.DetectionPosition.CENTER -> actionLeft
            DetectionPipeline.DetectionPosition.RIGHT -> actionLeft
        }

        val c = Canvas()
        action.preview(c)

        var running = true

        while (isStarted && !isStopRequested && running) {
            robot.update()

            val p = TelemetryPacket()
            p.fieldOverlay().operations.addAll(c.operations)

            running = action.run(p)

            dash.sendTelemetryPacket(p)
        }

    }
}