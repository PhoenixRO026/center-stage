package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.lib.hardware.servo.CachedServo.Companion.cachedServo
import org.firstinspires.ftc.teamcode.lib.hardware.servo.RangedCachedServo.Companion.rangedCachedServo
import org.firstinspires.ftc.teamcode.lib.hardware.servo.RangedServo.Companion.rangedServo
import org.firstinspires.ftc.teamcode.lib.hardware.servo.RangedSpeedCachedServo.Companion.rangedSpeedCachedServo
import org.firstinspires.ftc.teamcode.lib.hardware.servo.RangedSpeedServo.Companion.rangedSpeedServo
import org.firstinspires.ftc.teamcode.lib.hardware.servo.SimpleServo.Companion.simpleServo
import org.firstinspires.ftc.teamcode.lib.hardware.servo.SpeedCachedServo.Companion.speedCachedServo
import org.firstinspires.ftc.teamcode.lib.hardware.servo.SpeedServo.Companion.speedServo

class TestOp : LinearOpMode() {
    override fun runOpMode() {
        val simple = hardwareMap.simpleServo("a")

        val cached = hardwareMap.cachedServo("a")

        val speed = hardwareMap.speedServo("a")

        val ranged = hardwareMap.rangedServo("a")

        val speedCached = hardwareMap.speedCachedServo("a")

        val rangedCached = hardwareMap.rangedCachedServo("a")

        val rangedSpeed = hardwareMap.rangedSpeedServo("a")

        val rangedSpeedCached = hardwareMap.rangedSpeedCachedServo("a")
    }
}