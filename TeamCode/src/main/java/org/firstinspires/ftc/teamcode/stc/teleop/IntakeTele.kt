package org.firstinspires.ftc.teamcode.stc.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.stc.robot.Intake

@TeleOp
class IntakeTele : LinearOpMode(){
    override fun runOpMode() {
        val intake = Intake(hardwareMap)
        waitForStart()

        while(opModeIsActive()){
            intake.power = gamepad1.left_stick_y
        }
    }

}