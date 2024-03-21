package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.lib.units.DeltaTime
import org.firstinspires.ftc.teamcode.systems.Intake
import org.firstinspires.ftc.teamcode.systems.Intake.Companion.intake

@TeleOp
class StackTest : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val intake = hardwareMap.intake()

        intake.position = Intake.IntakeConfig.aboveFirstStack

        val dtime = DeltaTime()

        waitForStart()

        while (opModeIsActive()) {
            val deltaTime = dtime.calculateDeltaTime()

            if (gamepad1.a) {
                intake.position = Intake.IntakeConfig.aboveFirstStack
            } else if (gamepad1.b) {
                intake.position = Intake.IntakeConfig.firstStack
            }

            if (gamepad1.dpad_up) {
                intake.position += deltaTime.s
            } else if (gamepad1.dpad_down) {
                intake.position -= deltaTime.s
            }

            telemetry.addData("intake pos", intake.position)
            telemetry.update()
        }
    }
}