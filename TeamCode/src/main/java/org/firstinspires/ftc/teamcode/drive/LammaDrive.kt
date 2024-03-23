package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.gamepad.ButtonReader
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.lib.hardware.controlHub
import org.firstinspires.ftc.teamcode.lib.hardware.expansionHub
import org.firstinspires.ftc.teamcode.lib.opmode.MultiThreadOpMode
import org.firstinspires.ftc.teamcode.lib.units.DeltaTime
import org.firstinspires.ftc.teamcode.lib.units.rotate
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.systems.Arm.Companion.arm
import org.firstinspires.ftc.teamcode.systems.Box
import org.firstinspires.ftc.teamcode.systems.Box.Companion.box
import org.firstinspires.ftc.teamcode.systems.Intake.Companion.intake
import org.firstinspires.ftc.teamcode.systems.Lift.Companion.lift
import org.firstinspires.ftc.teamcode.systems.Plane.Companion.plane

@Photon
@TeleOp
open class LammaDrive : MultiThreadOpMode() {
    override fun sideRunOpMode() {}

    private val lift by opModeLazy {
        hardwareMap.lift()
    }

    private val box by opModeLazy {
        hardwareMap.box()
    }

    private val intake by opModeLazy {
        hardwareMap.intake()
    }

    private val arm by opModeLazy {
        hardwareMap.arm()
    }

    val drive by opModeLazy {
        MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
    }

    private val plane by opModeLazy {
        hardwareMap.plane()
    }

    private var speed: Double = 1.0

    private val deltaTimeCalc = DeltaTime()

    override fun mainRunOpMode() {
        val dash = FtcDashboard.getInstance()

        telemetry = MultipleTelemetry(telemetry, dash.telemetry)

        val controlHub = hardwareMap.controlHub()
        val expansionHub = hardwareMap.expansionHub()

        controlHub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        expansionHub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL

        val intakeToggle = ToggleButtonReader {
            gamepad2.left_stick_button
        }

        val scoreButton = ButtonReader {
            gamepad2.right_bumper
        }

        waitForStart()

        while (isStarted && !isStopRequested) {
            val deltaTime = deltaTimeCalc.calculateDeltaTime()

            controlHub.clearBulkCache()
            expansionHub.clearBulkCache()

            intakeToggle.readValue()
            scoreButton.readValue()

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

            if (gamepad1.y) {
                drive.imu.resetYaw()
            }

            val liftPower = -gamepad2.right_stick_y.toDouble()

            lift.power = liftPower

            if (gamepad2.back) {
                lift.hang()
            }

            if (gamepad2.start) {
                lift.unhang()
            }

            if (gamepad2.dpad_up) {
                arm.position += 0.1 * deltaTime.s
            }
            if (gamepad2.dpad_down) {
                arm.position -= 0.1 * deltaTime.s
            }
            if (gamepad2.dpad_right) {
                box.position += 0.1 * deltaTime.s
            }
            if (gamepad2.dpad_left) {
                box.position -= 0.1 * deltaTime.s
            }

            if (gamepad2.left_bumper) {
                box.intakePos()
                arm.intakePos()
            }

            if (scoreButton.wasJustPressed()) {
                if (box.targetPosition == Box.BoxConfig.scorePos) {
                    box.scoreLongPos()
                } else {
                    box.scorePos()
                }
                arm.scorePos()
            }

            box.power = -gamepad2.left_stick_y.toDouble()
            intake.power = -gamepad2.left_stick_y.toDouble()

            if (gamepad2.x) {
                intake.power = -1.0
            }

            if (intakeToggle.wasJustReleased()) {
                if (intakeToggle.state) {
                    intake.position = 1.0
                } else {
                    intake.position = 0.0
                }
            }

            if (gamepad2.left_stick_x <= -0.7 || gamepad2.left_trigger >= 0.3) {
                intake.goDown()
            } else if (gamepad2.left_stick_x >= 0.7 || gamepad2.right_trigger >= 0.3) {
                intake.goUp()
            }

            if (gamepad2.b) {
                plane.launch()
            }

            plane.update()

            arm.update()
            box.update()
            intake.update()
            lift.update()

            telemetry.addData("lift power", liftPower)

            telemetry.update()
        }
    }

    fun driveRobotCentric(forward: Double, strafe: Double, turn: Double) = drive.setDrivePowers(
        PoseVelocity2d(
        Vector2d(
            forward * speed,
            -strafe * speed
        ),
        -turn * speed
    )

    )
    open fun driveFieldCentric(forward: Double, strafe: Double, turn: Double) {
        val newInputVec = Vector2d(forward, strafe).rotate(drive.pose.heading.log())
        driveRobotCentric(newInputVec.x, newInputVec.y, turn)
    }
}