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
            lift.power = -gamepad1.right_stick_y

            if(gamepad1.back) lift.hang()
            if(gamepad1.start) lift.unhang()

            lift.update()

            telemetry.addData("lift power", lift.power)
            telemetry.addData("left pos", lift.leftLiftMotor.currentPosition)
            telemetry.addData("right pos", lift.rightLiftMotor.currentPosition)
            telemetry.addData("left target", lift.leftLiftMotor.targetPosition)
            telemetry.addData("right target", lift.rightLiftMotor.targetPosition)
            telemetry.update()
        }
    }


}