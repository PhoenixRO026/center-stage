package org.firstinspires.ftc.teamcode.stc.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.stc.robot.Claw

@TeleOp
class ClawTele : LinearOpMode() {
    override fun runOpMode() {
        val claw = Claw(hardwareMap)

        var prevTime: Long
        var deltaTime: Long
        var now: Long

        waitForStart()

        prevTime = System.currentTimeMillis()

        while (opModeIsActive()){
            now = System.currentTimeMillis()
            deltaTime = now - prevTime
            prevTime = now


            claw.leftFinger = if (gamepad1.a) 1.0 else 0.0

            if(gamepad1.b){
                claw.tilt += 0.01 * deltaTime
            } else if(gamepad1.x){
                claw.tilt -= 0.01 * deltaTime
            }

        }
    }
}