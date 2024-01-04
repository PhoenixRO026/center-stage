package org.firstinspires.ftc.teamcode.evenimente.liga_wonder

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.systems.Intake

@TeleOp
class IntakeTest : LinearOpMode() {
    override fun runOpMode() {
        val intake = Intake(hardwareMap)

        waitForStart()

        while (opModeIsActive()) {
            intake.power = (gamepad1.right_trigger - gamepad1.left_trigger).toDouble()
        }
    }
}