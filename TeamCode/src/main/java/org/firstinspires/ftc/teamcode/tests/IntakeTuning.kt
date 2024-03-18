package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.lib.units.DeltaTime
import org.firstinspires.ftc.teamcode.systems.Arm.Companion.arm
import org.firstinspires.ftc.teamcode.systems.Box.Companion.box
import org.firstinspires.ftc.teamcode.systems.Intake.Companion.intake

@TeleOp
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
            
            telemetry.addData("intake position", intake.position)
            telemetry.addData("intake unscaled position", intake.angleServo.unscaledPosition)
            telemetry.update()
        }
    }
}