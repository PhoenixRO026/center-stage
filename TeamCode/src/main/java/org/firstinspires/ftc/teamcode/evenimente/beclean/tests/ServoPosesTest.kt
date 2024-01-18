package org.firstinspires.ftc.teamcode.evenimente.beclean.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.ARM_PIXEL_DROP_POSE
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.ARM_RAMP_POS
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.ARM_SCORE_POS
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.systems.Arm2
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.CLAW_PIXEL_DROP_ANGLE
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.CLAW_RAMP_ANGLE
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.CLAW_RANGE
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.CLAW_SCORE_ANGLE
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.FINGERS_RANGE
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.systems.Claw2
import org.firstinspires.ftc.teamcode.evenimente.beclean.robot.systems.Lift2

@TeleOp
class ServoPosesTest : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val arm = Arm2(hardwareMap, telemetry, ARM_RAMP_POS..1.0)
        val lift = Lift2(hardwareMap, telemetry)
        lift.breakingMode = DcMotor.ZeroPowerBehavior.FLOAT
        val claw = Claw2(hardwareMap, telemetry, FINGERS_RANGE, CLAW_RANGE)

        val armPos = 0.0
        val clawPos = CLAW_RAMP_ANGLE

        arm.position = armPos
        arm.goToTargetNow()
        claw.angle = clawPos
        claw.goToAngleNow()

        var previousTime = time
        var deltaTime = 1.0

        while (opModeInInit()) {
            deltaTime = (time - previousTime) * 60.0
            previousTime = time

            arm.update(deltaTime)
            claw.update(deltaTime)
        }

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                arm.position += 0.005 * deltaTime
            }

            if (gamepad1.dpad_down) {
                arm.position -= 0.005 * deltaTime
            }

            if (gamepad1.y) {
                claw.angle += 0.005 * deltaTime
            }

            if (gamepad1.a) {
                claw.angle -= 0.005 * deltaTime
            }

            if (gamepad1.right_bumper) {
                arm.position = ARM_SCORE_POS
                claw.angle = CLAW_SCORE_ANGLE
            }

            if (gamepad1.left_bumper) {
                arm.position = 0.0
                claw.angle = CLAW_RAMP_ANGLE
            }

            if (gamepad1.x) {
                arm.position = ARM_PIXEL_DROP_POSE
                claw.angle = CLAW_PIXEL_DROP_ANGLE
            }

            arm.update(deltaTime)
            claw.update(deltaTime)

            telemetry.addData("arm pos", arm.position)
            telemetry.addData("arm real pos", arm.realPosition)
            telemetry.addData("claw pos", claw.angle)
            telemetry.addData("claw real angle", claw.realAngle)
            telemetry.addData("lift", lift.leftPosition)
            telemetry.addData("deltaTime", deltaTime)
            telemetry.update()

            deltaTime = (time - previousTime) * 60.0
            previousTime = time
        }

    }
}