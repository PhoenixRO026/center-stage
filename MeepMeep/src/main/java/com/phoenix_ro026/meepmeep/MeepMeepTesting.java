package com.phoenix_ro026.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep meepMeep = new MeepMeep(800);

        /*RoadRunnerBotEntity bot1 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -61, Math.toRadians(90)))
                                /// traj3
                                *//*.splineTo(new Vector2d(22, -34), Math.toRadians(45))
                                .waitSeconds(1)
                                .turn(Math.toRadians(-45))
                                .forward(24)
                                .waitSeconds(1)
                                .strafeLeft(15) //traj3*//*
                                ///traj1
                                *//*.splineTo(new Vector2d(2, -34), Math.toRadians(125))
                                .waitSeconds(1)
                                .turn(Math.toRadians(-125))
                                .lineTo(new Vector2d(2 + 42, -34))
                                //.forward(42)
                                .lineTo(new Vector2d(2 + 42, -35 + 18))
                                //.strafeLeft(18)
                                ///traj1*//*
                                *//*///traj2
                                .forward(29)
                                .turn(Math.toRadians(-90))
                                .forward(38)
                                .strafeLeft(18)
                                ///traj 2*//*

                                *//*.splineTo(new Vector2d())*//*
                                *//*.forward(25)
                                .turn(Math.toRadians(60))
                                .waitSeconds(1)
                                .splineTo(new Vector2d(47,-35),Math.toRadians(0))
                                .waitSeconds(0.5)
                                .strafeLeft(18)*//*
                                .build()
                );*/

        RoadRunnerBotEntity bot1 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 61, Math.toRadians(90)))
                                .back(20)
                                .turn(Math.toRadians(45))
                                .forward(5)
                                .waitSeconds(1)
                                .back(5)
                                .turn(Math.toRadians(45))
                                .back(24)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot1)
                .start();
    }
}