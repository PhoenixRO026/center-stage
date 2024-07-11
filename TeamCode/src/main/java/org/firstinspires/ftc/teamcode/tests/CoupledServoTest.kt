package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.lib.hardware.servo.RangedServo.Companion.rangedServo
import org.firstinspires.ftc.teamcode.lib.hardware.servo.RangedSpeedCachedServo.Companion.rangedSpeedCachedServo
import org.firstinspires.ftc.teamcode.lib.hardware.servo.SimpleServo.Companion.simpleServo

@Disabled
@TeleOp(group = "Debug")
class CoupledServoTest : LinearOpMode(){
    override fun runOpMode() {
        val servo2 = hardwareMap.rangedServo("servo2", range = 0.5..1.0)
        val servo3 = hardwareMap.simpleServo("servo3")
        val servo1 = hardwareMap.rangedSpeedCachedServo(
            deviceName = "servo1",
            cachingThreshold = 0.01,
            speed = 0.1,
            range = 0.0..0.5,
            coupledServos = listOf(servo2, servo3)
        )

        waitForStart()

        while (opModeIsActive()) {
            if (gamepad1.a) {
                servo1.targetPosition = 1.0
            }

            if (gamepad1.b) {
                servo1.targetPosition = 0.0
            }

            servo1.update()

            telemetry.addData("servo1 target pos", servo1.targetPosition)
            telemetry.addData("servo 1 cached pos", servo1.cachedPosition)
            telemetry.addData("servo1 unscaled target pos", servo1.unscaledTargetPosition)
            telemetry.addData("servo1 unscaled cached pos", servo1.unscaledCachedPosition)
            telemetry.addData("servo1 unscaled pos", servo1.unscaledPosition)
            telemetry.addData("servo2 unscaled pos", servo2.unscaledPosition)
            telemetry.addData("servo3 pos", servo3.position)
            telemetry.update()
        }
    }
}