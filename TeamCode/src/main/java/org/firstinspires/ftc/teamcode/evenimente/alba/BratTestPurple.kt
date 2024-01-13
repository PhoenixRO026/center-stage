package org.firstinspires.ftc.teamcode.evenimente.alba

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.evenimente.alba.robot.ARM_PIXEL_DROP_POSE
import org.firstinspires.ftc.teamcode.evenimente.alba.robot.Arm2
import org.firstinspires.ftc.teamcode.evenimente.alba.robot.CLAW_PIXEL_DROP_POSE
import org.firstinspires.ftc.teamcode.evenimente.alba.robot.Claw2
import org.firstinspires.ftc.teamcode.evenimente.alba.robot.Drive2
import org.firstinspires.ftc.teamcode.evenimente.alba.robot.Intake2
import org.firstinspires.ftc.teamcode.evenimente.alba.robot.Lift2
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.ARM_RAMP_POS
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.ARM_SCORE_POS
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.CLAW_RAMP_POS
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.CONFIG
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.FINGERS_RANGE
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.LEFT_ARM_SERVO_RANGE
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.RIGHT_ARM_SERVO_RANGE
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.hardware.ServoEx
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.systems.Arm
import org.firstinspires.ftc.teamcode.evenimente.liga_wonder.robot.systems.Camera

@TeleOp
class BratTestPurple : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val arm = Arm2(hardwareMap, telemetry, ARM_RAMP_POS..1.0)
        val lift = Lift2(hardwareMap, telemetry)
        val claw = Claw2(hardwareMap, telemetry, FINGERS_RANGE)

        val armPos = ARM_RAMP_POS
        val clawPos = CLAW_RAMP_POS

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
                arm.position = ARM_PIXEL_DROP_POSE
                claw.angle = CLAW_PIXEL_DROP_POSE
            }

            arm.update(deltaTime)
            claw.update(deltaTime)

            telemetry.addData("arm", armPos)
            telemetry.addData("arm", armPos)
            telemetry.addData("lift", lift.leftPosition)
            telemetry.addData("deltaTime", deltaTime)
            telemetry.update()

            deltaTime = (time - previousTime) * 60.0
            previousTime = time
        }

    }
}