package org.firstinspires.ftc.teamcode.stc.teleop

import com.acmerobotics.roadrunner.now
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.stc.robot.Arm
import org.firstinspires.ftc.teamcode.stc.robot.Claw

@TeleOp
class ArmClawTuning: LinearOpMode() {
    override fun runOpMode() {
        val arm = Arm(hardwareMap)
        val claw = Claw(hardwareMap)

        arm.pos = Arm.intakePos
        claw.tilt = Claw.intakeTilt

        var previousTime: Double
        var deltaTime: Double
        var now: Double

        waitForStart()

        previousTime = now()

        while (opModeIsActive()) {
            now = now()
            deltaTime = now - previousTime
            previousTime = now

            if (gamepad1.dpad_up) {
                arm.pos += 0.1 * deltaTime
            } else if (gamepad1.dpad_down) {
                arm.pos -= 0.1 * deltaTime
            }

            if (gamepad1.y) {
                claw.tilt += 0.1 * deltaTime
            } else if (gamepad1.a) {
                claw.tilt -= 0.1 * deltaTime
            }

            if (gamepad1.left_bumper) {
                claw.tilt = Claw.intakeTilt
                arm.pos = Arm.intakePos
            } else if (gamepad1.right_bumper) {
                claw.tilt = Claw.scoreTilt
                arm.pos = Arm.scorePos
            }

            telemetry.addData("arm pos", arm.pos)
            telemetry.addData("claw tilt", claw.tilt)
            telemetry.update()
        }
    }
}