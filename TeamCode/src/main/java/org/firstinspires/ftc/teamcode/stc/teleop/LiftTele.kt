package org.firstinspires.ftc.teamcode.stc.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.stc.robot.LiftForTele

@TeleOp
class LiftTele : LinearOpMode(){
    override fun runOpMode() {
        val lift = LiftForTele(hardwareMap)

        waitForStart()

        while(opModeIsActive()){
            lift.power = gamepad2.left_trigger

            if(gamepad2.a) lift.hang()
            if(gamepad2.b) lift.unhang()

        }
    }


}