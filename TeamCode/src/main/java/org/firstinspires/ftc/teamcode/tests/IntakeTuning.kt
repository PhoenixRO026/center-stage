package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.phoenix.phoenixlib.units.DeltaTime
import org.firstinspires.ftc.teamcode.systems.Intake.Companion.intake

//@Disabled
@TeleOp(group = "Debug")
class IntakeTuning : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        
        val intake = hardwareMap.intake()
        
        val time = DeltaTime()
        
        waitForStart()
        
        while (opModeIsActive()) {
            val dts = time.calculateDeltaTime().s

            if (gamepad1.dpad_up) {
                intake.position += dts
            } else if (gamepad1.dpad_down) {
                intake.position -= dts
            }

            intake.power = gamepad1.left_stick_x.toDouble()
            
            telemetry.addData("intake position", intake.position)
            telemetry.addData("intake unscaled position", intake.angleServo.unscaledPosition)
            telemetry.update()
        }
    }
}