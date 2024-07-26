package org.firstinspires.ftc.teamcode.stc.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.stc.robot.Arm
@TeleOp
class ArmTele : LinearOpMode() {
    override fun runOpMode() {
        val arm = Arm(hardwareMap)

        var prevTime: Long
        var deltaTime: Long
        var now: Long

        waitForStart()

        prevTime = System.currentTimeMillis()

        while (opModeIsActive()) {
            now = System.currentTimeMillis()
            deltaTime = now - prevTime
            prevTime = now

            if(gamepad1.b){
                arm.pos += 0.01 * deltaTime
            } else if(gamepad1.x){
                arm.pos -= 0.01 * deltaTime
            }

            telemetry.addData("Arm Position", arm.pos)
            telemetry.update()
        }
    }


}