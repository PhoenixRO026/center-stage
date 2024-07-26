package org.firstinspires.ftc.teamcode.stc.teleop

import com.acmerobotics.roadrunner.now
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.stc.robot.Arm
import kotlin.math.E

@TeleOp
class ArmTele : LinearOpMode() {
    override fun runOpMode() {
        val arm = Arm(hardwareMap)

        var prevTime: Double
        var deltaTime: Double
        var now: Double

        arm.pos = 0.5

        waitForStart()

        prevTime = now()

        while (opModeIsActive()) {
            now = now()
            deltaTime = now - prevTime
            prevTime = now

            if(gamepad1.b){
                arm.pos += 50 * deltaTime
            } else if(gamepad1.x){
                arm.pos -= 50 * deltaTime
            }

            telemetry.addData("Arm Position", arm.pos)
            telemetry.addData("delta time", deltaTime)
            telemetry.update()
        }
    }


}