package com.phoenix_ro026.meepmeep

import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.phoenix.phoenixlib.units.Pose
import com.phoenix.phoenixlib.units.cm
import com.phoenix.phoenixlib.units.deg
import com.phoenix.phoenixlib.units.ex
import com.phoenix.phoenixlib.units.inch
import com.phoenix.phoenixlib.units.s
import com.phoenix.phoenixlib.units.tile

fun MeepMeep.criBots(): MeepMeep {

    tile = 61.cm

    val midBot = DefaultBotBuilder(this)
        .setConstraints(85.0, 80.0, 4.0, 4.0, 13.0)
        .setDimensions(40.cm.inch, 43.cm.inch)
        .build()

    val leftBot = DefaultBotBuilder(this)
        .setConstraints(85.0, 80.0, 4.0, 4.0, 13.0)
        .setDimensions(40.cm.inch, 43.cm.inch)
        .build()

    val rightBot = DefaultBotBuilder(this)
        .setConstraints(85.0, 80.0, 4.0, 4.0, 13.0)
        .setDimensions(40.cm.inch, 43.cm.inch)
        .build()

    val stackWait = 0.4.s
    val boardWait = 0.7.s
    val purpleWait = 0.4.s

    val startPose = Pose(-2.5.tile, -2.5.tile - 1.inch, -90.deg)

    val midPurplePixel = Pose(-2.5.tile, -0.5.tile - 4.inch, -90.deg)
    val leftPurplePixel = Pose(-3.tile, -1.tile, -90.deg)
    val rightPurplePixel = Pose(-2.tile, -1.tile, -90.deg)

    val midStacky = Pose(-3.5.tile, -0.5.tile, 180.deg)
    val leftStacky = midStacky
    val rightStacky = midStacky

    val midTransition = Pose(2.tile, -0.5.tile, 180.deg)
    val leftTransition = midTransition
    val rightTransition = midTransition

    val midCenterBoard = Pose(3.tile + 3.inch, -1.5.tile, 180.deg)
    val leftLeftBoard = Pose(3.tile + 3.inch, -1.5.tile + 6.inch, 180.deg)
    val rightRightBoard = Pose(3.tile + 3.inch, -1.5.tile - 6.inch, 180.deg)

    val midBoardAproachAngle = -30.deg
    val leftBoardAproachAngle = midBoardAproachAngle
    val rightBoardAproachAngle = midBoardAproachAngle

    val midBoardLeavingAngle = 150.deg
    val leftBoardLeavingAngle = midBoardLeavingAngle
    val rightBoardLeavingAngle = midBoardLeavingAngle

    val midAction = midBot.drive.actionBuilder(startPose.pose2d).ex()
        .strafeToLinearHeading(midPurplePixel.position, midPurplePixel.heading)
        .waitSeconds(purpleWait)
        .setTangent(90.deg)
        .splineToLinearHeading(midStacky, 180.deg)
        .waitSeconds(stackWait)
        .setTangent(0.deg)
        .splineToConstantHeading(midTransition.position, 0.deg)
        .splineToConstantHeading(midCenterBoard.position, midBoardAproachAngle)
        .waitSeconds(boardWait)
        .setTangent(midBoardLeavingAngle)
        .splineToConstantHeading(midTransition.position, 180.deg)
        .splineToConstantHeading(midStacky.position, 180.deg)
        .waitSeconds(stackWait)
        .setTangent(0.deg)
        .splineToConstantHeading(midTransition.position, 0.deg)
        .splineToConstantHeading(midCenterBoard.position, midBoardAproachAngle)
        .build()

    val leftAction = leftBot.drive.actionBuilder(startPose.pose2d).ex()
        .strafeToLinearHeading(leftPurplePixel.position, leftPurplePixel.heading)
        .waitSeconds(purpleWait)
        .setTangent(90.deg)
        .splineToLinearHeading(leftStacky, 180.deg)
        .waitSeconds(stackWait)
        .setTangent(0.deg)
        .splineToConstantHeading(leftTransition.position, 0.deg)
        .splineToConstantHeading(leftLeftBoard.position, leftBoardAproachAngle)
        .waitSeconds(boardWait)
        .setTangent(leftBoardLeavingAngle)
        .splineToConstantHeading(leftTransition.position, 180.deg)
        .splineToConstantHeading(leftStacky.position, 180.deg)
        .waitSeconds(stackWait)
        .setTangent(0.deg)
        .splineToConstantHeading(leftTransition.position, 0.deg)
        .splineToConstantHeading(leftLeftBoard.position, leftBoardAproachAngle)
        .build()

    val rightAction = rightBot.drive.actionBuilder(startPose.pose2d).ex()
        .strafeToLinearHeading(rightPurplePixel.position, rightPurplePixel.heading)
        .waitSeconds(purpleWait)
        .setTangent(90.deg)
        .splineToLinearHeading(rightStacky, 180.deg)
        .waitSeconds(stackWait)
        .setTangent(0.deg)
        .splineToConstantHeading(rightTransition.position, 0.deg)
        .splineToConstantHeading(rightRightBoard.position, rightBoardAproachAngle)
        .waitSeconds(boardWait)
        .setTangent(rightBoardLeavingAngle)
        .splineToConstantHeading(rightTransition.position, 180.deg)
        .splineToConstantHeading(rightStacky.position, 180.deg)
        .waitSeconds(stackWait)
        .setTangent(0.deg)
        .splineToConstantHeading(rightTransition.position, 0.deg)
        .splineToConstantHeading(rightRightBoard.position, rightBoardAproachAngle)
        .build()

    midBot.runAction(midAction)
    leftBot.runAction(leftAction)
    rightBot.runAction(rightAction)

    addEntity(midBot)
    addEntity(leftBot)
    addEntity(rightBot)

    return this
}