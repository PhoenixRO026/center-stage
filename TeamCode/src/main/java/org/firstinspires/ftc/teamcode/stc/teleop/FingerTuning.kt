package org.firstinspires.ftc.teamcode.stc.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.stc.robot.Claw

@TeleOp
class FingerTuning: LinearOpMode() {
    override fun runOpMode() {
        val claw = Claw(hardwareMap)

        waitForStart()

        while (opModeIsActive()) {

            claw.leftFinger = gamepad1.right_trigger.toDouble()

            claw.rightFinger = gamepad1.left_trigger.toDouble()

            telemetry.addData("left finger", claw.leftFinger)
            telemetry.addData("right finger", claw.rightFinger)
            telemetry.update()
        }
    }
}