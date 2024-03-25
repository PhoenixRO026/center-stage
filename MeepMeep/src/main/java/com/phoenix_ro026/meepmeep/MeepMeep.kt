@file:JvmName("MeepMeep")

package com.phoenix_ro026.meepmeep

import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.Pose2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.phoenix.phoenixlib.units.Distance
import com.phoenix.phoenixlib.units.Distance2d
import com.phoenix.phoenixlib.units.TrajectoryActionBuilderEx
import com.phoenix.phoenixlib.units.ex

fun main() {
    System.setProperty("sun.java2d.opengl", "true")

    val meepMeep = MeepMeep(800)

    val stackWait = 0.5
    val boardWait = 0.5
    val purpleWait = 0.4

    val startPose = Pose2d(-36.0, -61.0, Math.toRadians(-90.0))

    val (position, heading) = Pose2d(-36.0, -12.0, Math.toRadians(-90.0))

    val (position1) = Pose2d(48.5, -35.0, Math.toRadians(180.0))

    val (position2) = Pose2d(48.5, -32.0, Math.toRadians(180.0))

    val (position3) = Pose2d(24.0, -12.0, Math.toRadians(180.0))

    val (position4) = Pose2d(-42.0, -12.0, Math.toRadians(180.0))

    val (position5, heading1) = Pose2d(-54.0, -12.0, Math.toRadians(180.0))

    val (position6, heading2) = Pose2d(-58.0, -12.0, Math.toRadians(180.0))

    val (position7) = Pose2d(-58.0, -24.0, Math.toRadians(180.0))

    val startPose2 = Pose2d(12.0, -61.0, Math.toRadians(-90.0))

    val (position8, heading3) = Pose2d(12.0, -12.0, Math.toRadians(-90.0))

    val middleYellowPixel2 = Pose2d(48.5, -35.0, Math.toRadians(180.0))

    val (position9) = Pose2d(48.5, -32.0, Math.toRadians(180.0))

    val (position10) = Pose2d(48.5, -60.0, Math.toRadians(180.0))

    val (position11) = Pose2d(24.0, -60.0, Math.toRadians(180.0))

    val (position12) = Pose2d(-30.0, -60.0, Math.toRadians(180.0))

    val (position13, heading4) = Pose2d(-54.0, -36.0, Math.toRadians(180.0))

    val (position14) = Pose2d(-58.0, -36.0, Math.toRadians(180.0))

    val myBot = DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(95.0, 90.0, 4.0, 4.0, 13.0)
            .build()

    val myBot2 = DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(85.0, 80.0, 4.0, 4.0, 13.0)
            .build()

    val kinematics = MecanumKinematics(13.0, 1.0)
    val speed60 = kinematics.WheelVelConstraint(60.0)

    val action = myBot.drive.actionBuilder(startPose).ex()
            .strafeToLinearHeading(position, heading)
            .waitSeconds(purpleWait)
            .strafeToSplineHeading(position5, heading1)
            .strafeToSplineHeading(position6, heading2)
            .waitSeconds(stackWait)
            .setTangent(Math.toRadians(0.0))
            .splineToConstantHeading(position4, Math.toRadians(0.0))
            .splineToConstantHeading(position3, Math.toRadians(0.0))
            .splineToConstantHeading(position2, Math.toRadians(-30.0), speed60)
            .waitSeconds(boardWait)
            .strafeTo(position1)
            .waitSeconds(boardWait)
            .setTangent(Math.toRadians(150.0))
            .splineToConstantHeading(position3, Math.toRadians(180.0), speed60)
            .splineToConstantHeading(position4, Math.toRadians(180.0))
            .splineToConstantHeading(position6, Math.toRadians(180.0))
            .waitSeconds(stackWait)
            .setTangent(Math.toRadians(0.0))
            .splineToConstantHeading(position4, Math.toRadians(0.0))
            .splineToConstantHeading(position3, Math.toRadians(0.0))
            .splineToConstantHeading(position2, Math.toRadians(-30.0), speed60)
            .waitSeconds(boardWait)
            .setTangent(Math.toRadians(150.0))
            .splineToConstantHeading(position3, Math.toRadians(180.0), speed60)
            .splineToConstantHeading(position4, Math.toRadians(180.0))
            .splineToConstantHeading(position6, Math.toRadians(180.0))
            .waitSeconds(stackWait)
            .setTangent(Math.toRadians(0.0))
            .splineToConstantHeading(position4, Math.toRadians(0.0))
            .splineToConstantHeading(position3, Math.toRadians(0.0))
            .splineToConstantHeading(position2, Math.toRadians(-30.0), speed60)
            .waitSeconds(boardWait)
            .setTangent(Math.toRadians(150.0))
            .splineToConstantHeading(position3, Math.toRadians(180.0), speed60)
            .splineToConstantHeading(position4, Math.toRadians(180.0))
            .splineToConstantHeading(position7, Math.toRadians(180.0), speed60)
            .waitSeconds(stackWait)
            .setTangent(Math.toRadians(0.0))
            .splineToConstantHeading(position4, Math.toRadians(0.0))
            .splineToConstantHeading(position3, Math.toRadians(0.0))
            .splineToConstantHeading(position2, Math.toRadians(-30.0), speed60)
            .waitSeconds(boardWait)
            .setTangent(Math.toRadians(150.0))
            .splineToConstantHeading(position3, Math.toRadians(180.0), speed60)
            .splineToConstantHeading(position4, Math.toRadians(180.0))
            .splineToConstantHeading(position7, Math.toRadians(180.0), speed60)
            .waitSeconds(stackWait)
            .setTangent(Math.toRadians(0.0))
            .splineToConstantHeading(position4, Math.toRadians(0.0))
            .splineToConstantHeading(position3, Math.toRadians(0.0))
            .splineToConstantHeading(position2, Math.toRadians(-30.0), speed60)
            .waitSeconds(boardWait)
            .build()

    val action2 = myBot.drive.actionBuilder(startPose2)
            .strafeToLinearHeading(position8, heading3)
            .waitSeconds(1.0)
            .setTangent(Math.toRadians(0.0))
            .splineToLinearHeading(middleYellowPixel2, Math.toRadians(-30.0))
            .waitSeconds(1.0)
            .setTangent(Math.toRadians(-150.0))
            .splineToConstantHeading(position11, Math.toRadians(180.0))
            .splineToConstantHeading(position12, Math.toRadians(180.0))
            .splineToConstantHeading(position14, Math.toRadians(180.0))
            .waitSeconds(1.0)
            .setTangent(Math.toRadians(0.0))
            .splineToConstantHeading(position12, Math.toRadians(0.0))
            .splineToConstantHeading(position11, Math.toRadians(0.0))
            .splineToConstantHeading(position9, Math.toRadians(30.0))
            .waitSeconds(1.0)
            .setTangent(Math.toRadians(-150.0))
            .splineToConstantHeading(position11, Math.toRadians(180.0))
            .splineToConstantHeading(position12, Math.toRadians(180.0))
            .splineToConstantHeading(position14, Math.toRadians(180.0))
            .waitSeconds(1.0)
            .setTangent(Math.toRadians(0.0))
            .splineToConstantHeading(position12, Math.toRadians(0.0))
            .splineToConstantHeading(position11, Math.toRadians(0.0))
            .splineToConstantHeading(position9, Math.toRadians(30.0))
            .waitSeconds(1.0)
            .setTangent(Math.toRadians(-150.0))
            .splineToConstantHeading(position11, Math.toRadians(180.0))
            .splineToConstantHeading(position12, Math.toRadians(180.0))
            .splineToConstantHeading(position14, Math.toRadians(180.0))
            .waitSeconds(1.0)
            .setTangent(Math.toRadians(0.0))
            .splineToConstantHeading(position12, Math.toRadians(0.0))
            .splineToConstantHeading(position11, Math.toRadians(0.0))
            .splineToConstantHeading(position9, Math.toRadians(30.0))
            .waitSeconds(1.0)
            .strafeTo(position10)
            .build()

    myBot.runAction(action)

    myBot2.runAction(action2)

    meepMeep.setBackground(Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
            .setDarkMode(false)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot) //.addEntity(myBot2)
            .start()
}