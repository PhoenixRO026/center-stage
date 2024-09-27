package org.firstinspires.ftc.teamcode.stc.teleop

import com.acmerobotics.roadrunner.now
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class PlaneTele : LinearOpMode() {
    override fun runOpMode() {
        val plane = Plane(hardwareMap)

        var prevTime: Double
        var deltaTime: Double
        var now: Double

        waitForStart()

        prevTime = now()

        while (opModeIsActive()){
            now = now()
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