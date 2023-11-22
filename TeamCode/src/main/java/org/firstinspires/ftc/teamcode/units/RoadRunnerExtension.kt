@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.units

import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.VelConstraint



fun TrajectoryActionBuilder.test() {

}

fun TrajectoryActionBuilder.afterDisp(
    ds: Distance, a: Action
) = afterDisp(ds.toInches(), a)

fun TrajectoryActionBuilder.afterTime(
    dt: Duration,
    a: Action
) = afterTime(dt.toSeconds(), a)

fun TrajectoryActionBuilder.turn(
    angle: Rotation,
    turnConstraintsOverride: TurnConstraints? = null
) = turn(angle.toRadians(), turnConstraintsOverride)

fun TrajectoryActionBuilder.lineToX(
    posX: Distance,
    velConstraintOverride: VelConstraint? = null,
    accelConstraintOverride: AccelConstraint? = null
) = lineToX(posX.toInches(), velConstraintOverride, accelConstraintOverride)

fun TrajectoryActionBuilder.lineToXConstantHeading(
    posX: Distance,
    velConstraintOverride: VelConstraint? = null,
    accelConstraintOverride: AccelConstraint? = null
) = lineToXConstantHeading(posX.toInches(), velConstraintOverride, accelConstraintOverride)

fun TrajectoryActionBuilder.lineToXLinearHeading(
    posX: Distance,
    heading: Rotation,
    velConstraintOverride: VelConstraint? = null,
    accelConstraintOverride: AccelConstraint? = null
) = lineToXLinearHeading(posX.toInches(), heading.toRadians(), velConstraintOverride, accelConstraintOverride)

fun TrajectoryActionBuilder.lineToXSplineHeading(
    posX: Distance,
    heading: Rotation,
    velConstraintOverride: VelConstraint? = null,
    accelConstraintOverride: AccelConstraint? = null
) = lineToXSplineHeading(posX.toInches(), heading.toRadians(), velConstraintOverride, accelConstraintOverride)

fun TrajectoryActionBuilder.lineToY(
    posX: Distance,
    velConstraintOverride: VelConstraint? = null,
    accelConstraintOverride: AccelConstraint? = null
) = lineToY(posX.toInches(), velConstraintOverride, accelConstraintOverride)

fun TrajectoryActionBuilder.lineToYConstantHeading(
    posX: Distance,
    velConstraintOverride: VelConstraint? = null,
    accelConstraintOverride: AccelConstraint? = null
) = lineToYConstantHeading(posX.toInches(), velConstraintOverride, accelConstraintOverride)

fun TrajectoryActionBuilder.lineToYLinearHeading(
    posX: Distance,
    heading: Rotation,
    velConstraintOverride: VelConstraint? = null,
    accelConstraintOverride: AccelConstraint? = null
) = lineToYLinearHeading(posX.toInches(), heading.toRadians(), velConstraintOverride, accelConstraintOverride)

fun TrajectoryActionBuilder.lineToYSplineHeading(
    posX: Distance,
    heading: Rotation,
    velConstraintOverride: VelConstraint? = null,
    accelConstraintOverride: AccelConstraint? = null
) = lineToYSplineHeading(posX.toInches(), heading.toRadians(), velConstraintOverride, accelConstraintOverride)

fun TrajectoryActionBuilder.setTangent(
    r: Rotation
) = setTangent(r.toRadians())

fun TrajectoryActionBuilder.splineTo(
    pos: Distance2d,
    tangent: Rotation,
    velConstraintOverride: VelConstraint? = null,
    accelConstraintOverride: AccelConstraint? = null
) = splineTo(pos.toVector2d(), tangent.toRadians(), velConstraintOverride, accelConstraintOverride)

fun TrajectoryActionBuilder.splineToConstantHeading(
    pos: Distance2d,
    tangent: Rotation,
    velConstraintOverride: VelConstraint? = null,
    accelConstraintOverride: AccelConstraint? = null
) = splineToConstantHeading(pos.toVector2d(), tangent.toRadians(), velConstraintOverride, accelConstraintOverride)
