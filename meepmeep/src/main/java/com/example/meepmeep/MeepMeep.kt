package com.example.meepmeep

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder

fun main() {
    System.setProperty("sun.java2d.opengl", "true")

    val meepMeep = MeepMeep(800)

    val myBot = DefaultBotBuilder(meepMeep)
        .build()

    val action = myBot.drive.actionBuilder(Pose2d(0.0, 0.0, 0.0))
        .splineTo(Vector2d(20.0, 20.0), Math.toRadians(90.0))
        .build()

    myBot.runAction(action)

    meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
        .setDarkMode(false)
        .setBackgroundAlpha(0.95f)
        .addEntity(myBot)
        .start()
}