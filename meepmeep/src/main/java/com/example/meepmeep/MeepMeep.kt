package com.example.meepmeep

import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder

fun main() {
    System.setProperty("sun.java2d.opengl", "true")

    val meepMeep = MeepMeep(800)

    val myBot = DefaultBotBuilder(meepMeep)
        .build()

    val startPose = Pose2d(12.0, -62.0, Math.toRadians(-90.0))
    val midPurple = Pose2d(12.0, -36.0, Math.toRadians(-90.0))
    val leftPurple = Pose2d(10.0, -34.0, Math.toRadians(-45.0))
    val rightPurple = Pose2d(24.0, -34.0, Math.toRadians(-90.0))
    val turnPoint = Pose2d(24.0, -60.0, Math.toRadians(-90.0))
    val midBoard = Pose2d(48.0, -36.0, Math.toRadians(180.0))
    val leftBoard = Pose2d(48.0, -30.0, Math.toRadians(180.0))
    val rightBoard = Pose2d(48.0, -42.0, Math.toRadians(180.0))

    val action = myBot.drive.actionBuilder(startPose)
        .setTangent(Math.toRadians(90.0))
        .splineTo(leftPurple.position, Math.toRadians(135.0))
        .setTangent(Math.toRadians(-45.0))
        .splineTo(turnPoint.position, Math.toRadians(-90.0))
        .setTangent(Math.toRadians(90.0))
        .setTangent(Math.toRadians(90.0))
        .splineTo(leftBoard.position, Math.toRadians(0.0))
        .waitSeconds(1.0)
        .setTangent(Math.toRadians(180.0))
        .lineToX(40.0)
        .setTangent(Math.toRadians(-90.0))
        .lineToY(-60.0)
        .build()

    myBot.runAction(action)

    meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
        .setDarkMode(false)
        .setBackgroundAlpha(0.95f)
        .addEntity(myBot)
        .start()
}