package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.lib.units.Pose
import org.firstinspires.ftc.teamcode.lib.units.cm
import org.firstinspires.ftc.teamcode.lib.units.deg
import org.firstinspires.ftc.teamcode.lib.units.ex
import org.firstinspires.ftc.teamcode.lib.units.rev
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive

@Disabled
@TeleOp(group = "tuning")
class TenTurnTest : LinearOpMode() {

    @Config
    data object TurnTest {
        @JvmField var turns = 10.0
    }

    override fun runOpMode() {
        val startPose = Pose(0.cm, 0.cm, 0.deg)

        val drive = MecanumDrive(hardwareMap, startPose.pose2d)

        val action = drive.actionBuilder(startPose.pose2d).ex()
                .turn(TurnTest.turns.rev)
                .build()

        waitForStart()

        runBlocking(action)
    }
}