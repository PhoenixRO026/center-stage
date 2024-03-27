@file:JvmName("MeepMeep")

package com.phoenix_ro026.meepmeep

import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.Pose2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.phoenix.phoenixlib.units.Pose
import com.phoenix.phoenixlib.units.cm
import com.phoenix.phoenixlib.units.deg
import com.phoenix.phoenixlib.units.ex
import com.phoenix.phoenixlib.units.inch
import com.phoenix.phoenixlib.units.s
import com.phoenix.phoenixlib.units.tile

fun main() {
    System.setProperty("sun.java2d.opengl", "true")

    val meepMeep = MeepMeep(800)

    val stackWait = 0.5.s
    val boardWait = 0.5.s
    val purpleWait = 0.4.s

    val myBot = DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        .setConstraints(85.0, 80.0, 4.0, 4.0, 13.0)
        .setDimensions(40.cm.inch, 43.cm.inch)
        .build()

    val kinematics = MecanumKinematics(13.0, 1.0)
    val speed60 = kinematics.WheelVelConstraint(60.0)

    val startPose =             Pose(-1.5.tile, -61.inch, -90.deg)

    val middlePurplePixel =     Pose(-36 .inch, -16.inch, -60.deg)
    val rightPurplePixel =      Pose(-32.inch, -35.inch, 0.deg)
    val leftPurplePixel =       Pose(-47.5.inch, -16.inch, -90.deg)

    val middleStacky1 =         Pose(-54.inch - 15.5.cm, -0.5.tile, 180.deg)
    val rightStacky1 =          Pose(-54.inch - 15.5.cm, -0.5.tile, 180.deg)
    val leftStacky1 =           Pose(-54.inch - 15.5.cm, -0.5.tile, 180.deg)

    val middlePreStacky1 =      middleStacky1 + 10.cm.x
    val rightPreStacky1 =       rightStacky1 + 10.cm.x
    val leftPreStacky1 =        leftStacky1 + 10.cm.x

    val middlePostStackRun1 =   Pose(-30.inch, -0.5.tile, 180.deg)
    val rightPostStackRun1 =    Pose(-30.inch, -0.5.tile, 180.deg)
    val leftPostStackRun1 =     Pose(-30.inch, -0.5.tile, 180.deg)

    val middlePreBoardRun1 =    Pose(18.inch, -0.5.tile, 180.deg)
    val rightPreBoardRun1 =     Pose(18.inch, -0.5.tile, 180.deg)
    val leftPreBoardRun1 =      Pose(18.inch, -0.5.tile, 180.deg)

    val middleYellowPixel2 =    Pose(52.5.inch, -1.5.tile, 180.deg)
    val rightYellowPixel2 =     Pose(52.5.inch, -43.5.inch, 180.deg)
    val leftYellowPixel2 =      Pose(52.5.inch, -29.5.inch, 180.deg)

    val middlePreYellowPixel1 = middleYellowPixel2
    val rightPreYellowPixel1 =  rightYellowPixel2
    val leftPreYellowPixel1 =   leftYellowPixel2

    val middlePostBoardRun2 =   Pose(18.inch, -0.5.tile, 180.deg)
    val rightPostBoardRun2 =    Pose(18.inch, -0.5.tile, 180.deg)
    val leftPostBoardRun2 =     Pose(18.inch, -0.5.tile, 180.deg)

    val middlePreStackRun2 =    Pose(-30.inch, -0.5.tile, 180.deg)
    val rightPreStackRun2 =     Pose(-30.inch, -0.5.tile, 180.deg)
    val leftPreStackRun2 =      Pose(-30.inch, -0.5.tile, 180.deg)

    val middleStacky2 =         Pose(-54.inch - 15.5.cm, -0.5.tile, 180.deg)
    val rightStacky2 =          Pose(-54.inch - 15.5.cm, -0.5.tile, 180.deg)
    val leftStacky2 =           Pose(-54.inch - 15.5.cm, -0.5.tile, 180.deg)

    val middlePostStackRun2 =   Pose(-30.inch, -0.5.tile, 180.deg)
    val rightPostStackRun2 =    Pose(-30.inch, -0.5.tile, 180.deg)
    val leftPostStackRun2 =     Pose(-30.inch, -0.5.tile, 180.deg)

    val middlePreBoardRun2 =    Pose(18.inch, -0.5.tile, 180.deg)
    val rightPreBoardRun2 =     Pose(18.inch, -0.5.tile, 180.deg)
    val leftPreBoardRun2 =      Pose(18.inch, -0.5.tile, 180.deg)

    val middleYellowPixel3 =    middleYellowPixel2
    val rightYellowPixel3 =     rightYellowPixel2
    val leftYellowPixel3 =      leftYellowPixel2

    val middlePreYellowPixel2 = middleYellowPixel3
    val rightPreYellowPixel2 =  rightYellowPixel3
    val leftPreYellowPixel2 =   leftYellowPixel3

    val middlePostBoardRun3 =   Pose(18.inch, -0.5.tile, 180.deg)
    val rightPostBoardRun3 =    Pose(18.inch, -0.5.tile, 180.deg)
    val leftPostBoardRun3 =     Pose(18.inch, -0.5.tile, 180.deg)

    val middlePreStackRun3 =    Pose(-30.inch, -0.5.tile, 180.deg)
    val rightPreStackRun3 =     Pose(-30.inch, -0.5.tile, 180.deg)
    val leftPreStackRun3 =      Pose(-30.inch, -0.5.tile, 180.deg)

    val middleStacky3 =         Pose(-54.inch - 15.5.cm, -0.5.tile, 180.deg)
    val rightStacky3 =          Pose(-54.inch - 15.5.cm, -0.5.tile, 180.deg)
    val leftStacky3 =           Pose(-54.inch - 15.5.cm, -0.5.tile, 180.deg)

    val middlePreStacky3 =      Pose(-54.inch - 15.5.cm, -0.5.tile, 180.deg)
    val rightPreStacky3 =       Pose(-54.inch - 15.5.cm, -0.5.tile, 180.deg)
    val leftPreStacky3 =        Pose(-54.inch - 15.5.cm, -0.5.tile, 180.deg)

    val middlePostStackRun3 =   Pose(-30.inch, -0.5.tile, 180.deg)
    val rightPostStackRun3 =    Pose(-30.inch, -0.5.tile, 180.deg)
    val leftPostStackRun3 =     Pose(-30.inch, -0.5.tile, 180.deg)

    val middlePreBoardRun3 =    Pose(18.inch, -0.5.tile, 180.deg)
    val rightPreBoardRun3 =     Pose(18.inch, -0.5.tile, 180.deg)
    val leftPreBoardRun3 =      Pose(18.inch, -0.5.tile, 180.deg)

    val middleYellowPixel4 =    middleYellowPixel3
    val rightYellowPixel4 =     rightYellowPixel3
    val leftYellowPixel4 =      leftYellowPixel3

    val middlePreYellowPixel3 = middleYellowPixel4
    val rightPreYellowPixel3 =  rightYellowPixel4
    val leftPreYellowPixel3 =   leftYellowPixel4

    val middlePostBoardRun4 =   Pose(18.inch, -0.5.tile, 180.deg)
    val rightPostBoardRun4 =    Pose(18.inch, -0.5.tile, 180.deg)
    val leftPostBoardRun4 =     Pose(18.inch, -0.5.tile, 180.deg)

    val middlePreStackRun4 =    Pose(-30.inch, -0.5.tile, 180.deg)
    val rightPreStackRun4 =     Pose(-30.inch, -0.5.tile, 180.deg)
    val leftPreStackRun4 =      Pose(-30.inch, -0.5.tile, 180.deg)

    val middleStacky4 =         Pose(-54.inch - 15.5.cm, -1.tile, 180.deg)
    val rightStacky4 =          Pose(-54.inch - 15.5.cm, -1.tile, 180.deg)
    val leftStacky4 =           Pose(-54.inch - 15.5.cm, -1.tile, 180.deg)

    val middlePreStacky4 =      Pose(-54.inch - 15.5.cm, -0.5.tile, 180.deg)
    val rightPreStacky4 =       Pose(-54.inch - 15.5.cm, -0.5.tile, 180.deg)
    val leftPreStacky4 =        Pose(-54.inch - 15.5.cm, -0.5.tile, 180.deg)

    val middlePostStackRun4 =   Pose(-30.inch, -0.5.tile, 180.deg)
    val rightPostStackRun4 =    Pose(-30.inch, -0.5.tile, 180.deg)
    val leftPostStackRun4 =     Pose(-30.inch, -0.5.tile, 180.deg)

    val middlePreBoardRun4 =    Pose(18.inch, -0.5.tile, 180.deg)
    val rightPreBoardRun4 =     Pose(18.inch, -0.5.tile, 180.deg)
    val leftPreBoardRun4 =      Pose(18.inch, -0.5.tile, 180.deg)

    val middleYellowPixel5 =    middleYellowPixel4
    val rightYellowPixel5 =     rightYellowPixel4
    val leftYellowPixel5 =      leftYellowPixel4

    val middlePreYellowPixel4 = middleYellowPixel5
    val rightPreYellowPixel4 =  rightYellowPixel5
    val leftPreYellowPixel4 =   leftYellowPixel5

    val middleBoardAproachAngle = -30.deg
    val rightBoardAproachAngle = -30.deg
    val leftBoardAproachAngle = -30.deg

    val middleBoardLeavingAngle = 150.deg
    val rightBoardLeavingAngle = 150.deg
    val leftBoardLeavingAngle = 150.deg

    val action = myBot.drive.actionBuilder(startPose.pose2d).ex()
        .strafeToLinearHeading(middlePurplePixel.position, middlePurplePixel.heading)
        .waitSeconds(purpleWait)
        .setTangent(90.deg)
        .splineToLinearHeading(middleStacky1, 180.deg)
        .waitSeconds(stackWait)
        .setTangent(0.deg)
        .splineToConstantHeading(middlePostStackRun1.position, 0.deg)
        .splineToConstantHeading(middlePreBoardRun1.position, 0.deg)
        .splineToConstantHeading(leftPreYellowPixel1.position, leftBoardAproachAngle, speed60)
        .waitSeconds(boardWait)
        .strafeTo(middleYellowPixel2.position)
        .waitSeconds(boardWait)
        .setTangent(leftBoardLeavingAngle)
        .splineToConstantHeading(middlePostBoardRun2.position, 180.deg, speed60)
        .splineToConstantHeading(middlePreStackRun2.position, 180.deg)
        .splineToConstantHeading(middleStacky2.position, 180.deg)
        .waitSeconds(stackWait)
        .setTangent(0.deg)
        .splineToConstantHeading(middlePostStackRun2.position, 0.deg)
        .splineToConstantHeading(middlePreBoardRun2.position, 0.deg)
        .splineToConstantHeading(leftPreYellowPixel2.position, leftBoardAproachAngle, speed60)
        .waitSeconds(boardWait)
        .setTangent(leftBoardLeavingAngle)
        .splineToConstantHeading(middlePostBoardRun3.position, 180.deg, speed60)
        .splineToConstantHeading(middlePreStackRun3.position, 180.deg)
        .splineToConstantHeading(middleStacky3.position, 180.deg)
        .waitSeconds(stackWait)
        .setTangent(0.deg)
        .splineToConstantHeading(middlePostStackRun3.position, 0.deg)
        .splineToConstantHeading(middlePreBoardRun3.position, 0.deg)
        .splineToConstantHeading(leftPreYellowPixel3.position, leftBoardAproachAngle, speed60)
        .waitSeconds(boardWait)
        .setTangent(leftBoardLeavingAngle)
        .splineToConstantHeading(middlePostBoardRun4.position, 180.deg, speed60)
        .splineToConstantHeading(middlePreStackRun4.position, 180.deg)
        .splineToConstantHeading(middleStacky4.position, 180.deg)
        .waitSeconds(stackWait)
        .setTangent(0.deg)
        .splineToConstantHeading(middlePostStackRun4.position, 0.deg)
        .splineToConstantHeading(middlePreBoardRun4.position, 0.deg)
        .splineToConstantHeading(leftPreYellowPixel4.position, leftBoardAproachAngle, speed60)
        .waitSeconds(boardWait)
        .build()

    myBot.runAction(action)

    meepMeep.setBackground(Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
            .setDarkMode(false)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start()
}