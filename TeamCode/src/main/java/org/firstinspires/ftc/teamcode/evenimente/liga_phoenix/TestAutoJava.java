package org.firstinspires.ftc.teamcode.evenimente.liga_phoenix;

import static com.phoenix_ro026.phoenixlib.old_units.java.DistanceKt.cm;
import static com.phoenix_ro026.phoenixlib.old_units.java.RotationKt.deg;
import static com.phoenix_ro026.phoenixlib.old_units.java.RotationKt.rad;

import com.acmerobotics.roadrunner.Vector2d;
import com.phoenix_ro026.phoenixlib.old_units.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Disabled
@Autonomous
public class TestAutoJava extends LinearOpMode {
    Pose beginPose = new Pose(cm(0.0), cm(0.0), rad(0.0));
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(cm(30), cm(30)), deg(90));
    }
}
