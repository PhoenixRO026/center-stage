package com.phoenix_ro026.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

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

        /*RoadRunnerBotEntity bot1 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(37.5 / 2.54, 43.5 / 2.54)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 61, Math.toRadians(90)))
                                .setTangent(Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(-32 - 6, 36, Math.toRadians(45)), Math.toRadians(-90))
                                //.lineToLinearHeading(new Pose2d(-32, 36, Math.toRadians(135)))
                                .waitSeconds(1)
                                .turn(Math.toRadians(45))
                                .setTangent(Math.toRadians(-90))
                                .splineTo(new Vector2d(-54, 11), Math.toRadians(180))
                                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(
                                        new AngularVelocityConstraint(Math.toRadians(180)),
                                        new MecanumVelocityConstraint(5, 15)
                                )))
                                .splineTo(new Vector2d(-58, 11), Math.toRadians(180))
                                .resetVelConstraint()
                                .waitSeconds(1)
                                .setTangent(0)
                                .splineTo(new Vector2d(12, 11), 0)
                                .splineToSplineHeading(new Pose2d(20, 11, Math.toRadians(180)), 0)
                                .splineTo(new Vector2d(44, 34), 0)
                                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(
                                        new AngularVelocityConstraint(Math.toRadians(180)),
                                        new MecanumVelocityConstraint(5, 15)
                                )))
                                .splineTo(new Vector2d(48, 34), 0)
                                .build()
                );*/

        RoadRunnerBotEntity bot1 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(37.5 / 2.54, 43.5 / 2.54)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 61, Math.toRadians(90)))
                                .setTangent(Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(-36, 36, Math.toRadians(90)), Math.toRadians(-90))
                                //.lineToLinearHeading(new Pose2d(-32, 36, Math.toRadians(135)))
                                .waitSeconds(1)
                                .turn(Math.toRadians(135))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot1)
                .start();
    }
}