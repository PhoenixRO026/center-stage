package com.phoenix_ro026.meepmeep;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose = new Pose2d(-36, -61, Math.toRadians(-90));

        Pose2d middlePurplePixel = new Pose2d(-36, -12, Math.toRadians(-90));

        Pose2d middleYellowPixel = new Pose2d(48.5, -35, Math.toRadians(180));

        Pose2d leftYellowPixel = new Pose2d(48.5, -32, Math.toRadians(180));

        Pose2d middleRun1 = new Pose2d(24, -12, Math.toRadians(180));

        Pose2d middleRun2 = new Pose2d(-30, -12, Math.toRadians(180));

        Pose2d stacky = new Pose2d(-54, -12, Math.toRadians(180));

        Pose2d stacky2 = new Pose2d(-58, -12, Math.toRadians(180));

        Pose2d startPose2 = new Pose2d(12, -61, Math.toRadians(-90));

        Pose2d middlePurplePixel2 = new Pose2d(12, -12, Math.toRadians(-90));

        Pose2d middleYellowPixel2 = new Pose2d(48.5, -35, Math.toRadians(180));

        Pose2d leftYellowPixel2 = new Pose2d(48.5, -32, Math.toRadians(180));

        Pose2d middleRun12 = new Pose2d(24, -60, Math.toRadians(180));

        Pose2d middleRun22 = new Pose2d(-30, -60, Math.toRadians(180));

        Pose2d stacky12 = new Pose2d(-54, -36, Math.toRadians(180));

        Pose2d stacky22 = new Pose2d(-58, -36, Math.toRadians(180));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(85, 80, 4, 4, 13)
                .build();

        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(85, 80, 4, 4, 13)
                .build();

        Action action = myBot.getDrive().actionBuilder(startPose)
                .strafeToLinearHeading(middlePurplePixel.position, middlePurplePixel.heading)
                .waitSeconds(1)
                .strafeToSplineHeading(stacky.position, stacky.heading)
                .strafeToSplineHeading(stacky2.position, stacky2.heading)
                .waitSeconds(1)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(middleRun2.position, Math.toRadians(0))
                .splineToConstantHeading(middleRun1.position, Math.toRadians(0))
                .splineToConstantHeading(leftYellowPixel.position, Math.toRadians(-30))
                .waitSeconds(1)
                .strafeTo(middleYellowPixel.position)
                .waitSeconds(1)
                .setTangent(Math.toRadians(150))
                .splineToConstantHeading(middleRun1.position, Math.toRadians(180))
                .splineToConstantHeading(middleRun2.position, Math.toRadians(180))
                .splineToConstantHeading(stacky2.position, Math.toRadians(180))
                .waitSeconds(1)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(middleRun2.position, Math.toRadians(0))
                .splineToConstantHeading(middleRun1.position, Math.toRadians(0))
                .splineToConstantHeading(leftYellowPixel.position, Math.toRadians(-30))
                .waitSeconds(1)
                .setTangent(Math.toRadians(150))
                .splineToConstantHeading(middleRun1.position, Math.toRadians(180))
                .splineToConstantHeading(middleRun2.position, Math.toRadians(180))
                .splineToConstantHeading(stacky2.position, Math.toRadians(180))
                .waitSeconds(1)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(middleRun2.position, Math.toRadians(0))
                .splineToConstantHeading(middleRun1.position, Math.toRadians(0))
                .splineToConstantHeading(leftYellowPixel.position, Math.toRadians(-30))
                .waitSeconds(1)
                .setTangent(Math.toRadians(150))
                .splineToConstantHeading(middleRun1.position, Math.toRadians(180))
                .splineToConstantHeading(middleRun2.position, Math.toRadians(180))
                .splineToConstantHeading(stacky2.position, Math.toRadians(180))
                .waitSeconds(1)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(middleRun2.position, Math.toRadians(0))
                .splineToConstantHeading(middleRun1.position, Math.toRadians(0))
                .splineToConstantHeading(leftYellowPixel.position, Math.toRadians(-30))
                .waitSeconds(1)
                .build();

        Action action2 = myBot.getDrive().actionBuilder(startPose2)
                .strafeToLinearHeading(middlePurplePixel2.position, middlePurplePixel2.heading)
                .waitSeconds(1)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(middleYellowPixel2, Math.toRadians(-30))
                .waitSeconds(1)
                .setTangent(Math.toRadians(-150))
                .splineToConstantHeading(middleRun12.position, Math.toRadians(180))
                .splineToConstantHeading(middleRun22.position, Math.toRadians(180))
                .splineToConstantHeading(stacky22.position, Math.toRadians(180))
                .waitSeconds(1)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(middleRun22.position, Math.toRadians(0))
                .splineToConstantHeading(middleRun12.position, Math.toRadians(0))
                .splineToConstantHeading(leftYellowPixel2.position, Math.toRadians(30))
                .waitSeconds(1)
                .setTangent(Math.toRadians(-150))
                .splineToConstantHeading(middleRun12.position, Math.toRadians(180))
                .splineToConstantHeading(middleRun22.position, Math.toRadians(180))
                .splineToConstantHeading(stacky22.position, Math.toRadians(180))
                .waitSeconds(1)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(middleRun22.position, Math.toRadians(0))
                .splineToConstantHeading(middleRun12.position, Math.toRadians(0))
                .splineToConstantHeading(leftYellowPixel2.position, Math.toRadians(30))
                .waitSeconds(1)
                .setTangent(Math.toRadians(-150))
                .splineToConstantHeading(middleRun12.position, Math.toRadians(180))
                .splineToConstantHeading(middleRun22.position, Math.toRadians(180))
                .splineToConstantHeading(stacky22.position, Math.toRadians(180))
                .waitSeconds(1)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(middleRun22.position, Math.toRadians(0))
                .splineToConstantHeading(middleRun12.position, Math.toRadians(0))
                .splineToConstantHeading(leftYellowPixel2.position, Math.toRadians(30))
                .waitSeconds(1)
                .build();

        myBot.runAction(action);

        myBot2.runAction(action2);

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(myBot2)
                .start();
    }
}