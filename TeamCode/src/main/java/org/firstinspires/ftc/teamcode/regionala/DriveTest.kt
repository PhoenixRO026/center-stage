package org.firstinspires.ftc.teamcode.regionala

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.gamepad.ButtonReader
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.lib.opmode.MultiThreadOpMode
import org.firstinspires.ftc.teamcode.lib.units.Pose
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.cm
import org.firstinspires.ftc.teamcode.lib.units.deg
import org.firstinspires.ftc.teamcode.lib.units.ms
import org.firstinspires.ftc.teamcode.lib.units.rotate
import org.firstinspires.ftc.teamcode.lib.units.s
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDriveEx
import org.firstinspires.ftc.teamcode.robot.Arm
import org.firstinspires.ftc.teamcode.robot.Arm.Companion.arm
import org.firstinspires.ftc.teamcode.robot.Claw
import org.firstinspires.ftc.teamcode.robot.Claw.Companion.claw
import org.firstinspires.ftc.teamcode.robot.Intake.Companion.intake
import org.firstinspires.ftc.teamcode.robot.Lift.Companion.lift
import org.firstinspires.ftc.teamcode.robot.Plane.Companion.plane
import org.firstinspires.ftc.teamcode.robot.hardware.controlHub
import org.firstinspires.ftc.teamcode.robot.hardware.expansionHub


@Photon
@TeleOp
class DriveTest: MultiThreadOpMode() {
    private val drive by opModeLazy {
        MecanumDriveEx(hardwareMap, Pose(0.cm, 0.cm, 0.deg))
    }
    private val arm by opModeLazy {
        hardwareMap.arm()
    }
    private val claw by opModeLazy {
        hardwareMap.claw()
    }
    private val lift by opModeLazy {
        hardwareMap.lift()
    }
    private val intake by opModeLazy {
        hardwareMap.intake()
    }

    private val plane by opModeLazy {
        hardwareMap.plane()
    }

    private var speed = 1.0

    override fun sideRunOpMode() {
    }

    override fun mainRunOpMode() {
        var previousTime = System.currentTimeMillis().ms
        var deltaTime: Time

        val dash = FtcDashboard.getInstance()

        telemetry = MultipleTelemetry(telemetry, dash.telemetry)

        val controlHub = hardwareMap.controlHub()
        val expansionHub = hardwareMap.expansionHub()

        controlHub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        expansionHub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL

        val intakeToggle = ToggleButtonReader {
            gamepad2.left_stick_button
        }

        val launchButton = ButtonReader {
            gamepad2.b
        }

        waitForStart()

        while (isStarted && !isStopRequested) {
            val now = System.currentTimeMillis().ms
            deltaTime = now - previousTime
            previousTime = now

            controlHub.clearBulkCache()
            expansionHub.clearBulkCache()

            intakeToggle.readValue()
            launchButton.readValue()

            plane.update()

            if (launchButton.wasJustPressed()) {
                plane.launch()
            }

            drive.updatePoseEstimate()

            speed = if (gamepad1.left_trigger >= 0.2) {
                0.5
            } else {
                1.0
            }

            driveFieldCentric(
                -gamepad1.left_stick_y.toDouble(),
                gamepad1.left_stick_x.toDouble(),
                gamepad1.right_stick_x.toDouble()
            )

            lift.power = -gamepad2.right_stick_y.toDouble()

            if (gamepad1.y) {
                drive.imu.resetYaw()
            }

            if (gamepad2.back) {
                lift.hangAsync()
            }

            if (gamepad2.start) {
                lift.unhangAsync()
            }

            if (gamepad2.touchpad) {
                lift.goToRampAsync()
            }

            if (gamepad2.dpad_up) {
                arm.position += 0.1 * deltaTime.s
            }
            if (gamepad2.dpad_down) {
                arm.position -= 0.1 * deltaTime.s
            }
            if (gamepad2.y) {
                claw.clawPosition += 0.1 * deltaTime.s
            }
            if (gamepad2.a) {
                claw.clawPosition -= 0.1 * deltaTime.s
            }

            if (gamepad2.left_bumper) {
                claw.clawTargetPosition = Claw.rampPos
                arm.targetPosition = Arm.rampPos
            }

            if (gamepad2.right_bumper) {
                claw.clawTargetPosition = Claw.scorePos
                arm.targetPosition = Arm.scorePos
            }

            if (gamepad2.x) {
                claw.leftFingerPosition = Claw.fingerRampPos
                claw.rightFingerPosition = Claw.fingerRampPos
            } else {
                claw.leftFingerPosition = 1.0 - gamepad2.right_trigger.toDouble()
                claw.rightFingerPosition = 1.0 - gamepad2.left_trigger.toDouble()
            }

            intake.power = -gamepad2.left_stick_y.toDouble()

            if (intakeToggle.state) {
                intake.position = 1.0
            } else {
                intake.position = 0.0
            }

            arm.update(deltaTime)
            claw.update(deltaTime)

            telemetry.addData("delta time ms", deltaTime.ms)
            telemetry.addData("fps", 1.s / deltaTime)
            telemetry.addData("arm pos", arm.position)
            telemetry.addData("real arm pos", arm.realPosition)
            telemetry.addData("claw pos", claw.clawPosition)
            telemetry.update()
        }
    }

    private fun driveRobotCentric(forward: Double, strafe: Double, turn: Double) = drive.setDrivePowers(PoseVelocity2d(
        Vector2d(
            forward * speed,
            -strafe * speed
        ),
        -turn * speed
    ))
    private fun driveFieldCentric(forward: Double, strafe: Double, turn: Double) {
        val newInputVec = Vector2d(forward, strafe).rotate(drive.pose.heading.log())
        driveRobotCentric(newInputVec.x, newInputVec.y, turn)
    }
}