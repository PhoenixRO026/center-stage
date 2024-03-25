package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.phoenix.phoenixlib.units.DeltaTime
import org.firstinspires.ftc.teamcode.systems.Arm.Companion.arm
import org.firstinspires.ftc.teamcode.systems.Box.Companion.box

@TeleOp
class ArmBoxTuning : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        
        val box = hardwareMap.box()
        val arm = hardwareMap.arm()
        
        val time = DeltaTime()
        
        waitForStart()
        
        while (opModeIsActive()) {
            val dts = time.calculateDeltaTime().s

            if (gamepad1.dpad_up) {
                box.position += 0.1 * dts
            } else if (gamepad1.dpad_down) {
                box.position -= 0.1 * dts
            }

            if (gamepad1.y) {
                arm.position += 0.1 * dts
            } else if (gamepad1.a) {
                arm.position -= 0.1 * dts
            }


            if (gamepad1.left_bumper) {
                box.intakePos()
                arm.intakePos()
            } else if (gamepad1.right_bumper) {
                box.scorePos()
                arm.scorePos()
            }

            arm.update()
            box.update()
            
            telemetry.addData("box position", box.position)
            telemetry.addData("arm position", arm.position)
            telemetry.update()
        }
    }
}