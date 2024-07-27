package org.firstinspires.ftc.teamcode.stc.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.stc.robot.Claw
import org.firstinspires.ftc.teamcode.stc.robot.Plane

@TeleOp
class PlaneTele : LinearOpMode() {
    override fun runOpMode() {
        val plane = Plane(hardwareMap)

        var prevTime: Long
        var deltaTime: Long
        var now: Long

        waitForStart()

        prevTime = System.currentTimeMillis()

        while (opModeIsActive()){
            now = System.currentTimeMillis()
            deltaTime = now - prevTime
            prevTime = now


            if (gamepad1.dpad_up) {
                plane.tiltPos += 0.1 * deltaTime
            } else if (gamepad1.dpad_down) {
                plane.tiltPos -= 0.1 * deltaTime
            }

            if (gamepad1.y) {
                plane.launchPos += 0.1 * deltaTime
            } else if (gamepad1.a) {
                plane.launchPos -= 0.1 * deltaTime
            }

            telemetry.addData("tilt pos", plane.tiltPos)
            telemetry.addData("launch pos", plane.launchPos)
            telemetry.update()
        }
    }
}