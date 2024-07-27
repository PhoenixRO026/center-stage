package org.firstinspires.ftc.teamcode.stc.teleop

import com.acmerobotics.roadrunner.now
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.stc.robot.Claw

@TeleOp
class ClawTele : LinearOpMode() {
    override fun runOpMode() {
        val claw = Claw(hardwareMap)

        var prevTime: Double
        var deltaTime: Double
        var now: Double

        waitForStart()

        prevTime = now()

        while (opModeIsActive()){
            now = now()
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