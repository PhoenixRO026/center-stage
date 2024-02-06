package org.firstinspires.ftc.teamcode.regionala

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.lib.opmode.MultiThreadOpMode
import org.firstinspires.ftc.teamcode.lib.units.Time
import org.firstinspires.ftc.teamcode.lib.units.ms
import org.firstinspires.ftc.teamcode.lib.units.s
import org.firstinspires.ftc.teamcode.robot.Arm
import org.firstinspires.ftc.teamcode.robot.Arm.Companion.arm
import org.firstinspires.ftc.teamcode.robot.Claw
import org.firstinspires.ftc.teamcode.robot.Claw.Companion.claw
import org.firstinspires.ftc.teamcode.robot.hardware.controlHub
import org.firstinspires.ftc.teamcode.robot.hardware.expansionHub


@Photon
@TeleOp
class ScoreTest: MultiThreadOpMode() {
    private val arm by opModeLazy {
        hardwareMap.arm()
    }
    private val claw by opModeLazy {
        hardwareMap.claw()
    }

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

        waitForStart()

        while (isStarted && !isStopRequested) {
            val now = System.currentTimeMillis().ms
            deltaTime = now - previousTime
            previousTime = now

            controlHub.clearBulkCache()
            expansionHub.clearBulkCache()

            if (gamepad1.dpad_up) {
                arm.position += 0.1 * deltaTime.s
            }
            if (gamepad1.dpad_down) {
                arm.position -= 0.1 * deltaTime.s
            }
            if (gamepad1.y) {
                claw.clawPosition += 0.1 * deltaTime.s
            }
            if (gamepad1.a) {
                claw.clawPosition -= 0.1 * deltaTime.s
            }

            if (gamepad1.left_bumper) {
                claw.clawTargetPosition = Claw.rampPos
                arm.targetPosition = Arm.rampPos
            }

            if (gamepad1.right_bumper) {
                claw.clawTargetPosition = Claw.scorePos
                arm.targetPosition = Arm.scorePos
            }

            arm.update(deltaTime)
            claw.update(deltaTime)

            telemetry.addData("delta time ms", deltaTime.ms)
            telemetry.addData("fps", 1.s / deltaTime)
            telemetry.addData("arm pos", arm.position)
            telemetry.addData("claw pos", claw.clawPosition)
            telemetry.update()
        }
    }
}