package com.phoenix_ro026.phoenixlib.units

import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantFunction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.VelConstraint

class TrajectoryActionBuilderEx(private val builder: TrajectoryActionBuilder) {
    fun endTrajectory() = TrajectoryActionBuilderEx(builder.endTrajectory())

    /**
     * Stops the current trajectory (like [endTrajectory]) and adds action [a] next.
     */
    fun stopAndAdd(a: Action) = TrajectoryActionBuilderEx(builder.stopAndAdd(a))
    fun stopAndAdd(f: InstantFunction) = TrajectoryActionBuilderEx(builder.stopAndAdd(f))

    /**
     * Waits [t] seconds.
     */
    fun waitSeconds(t: Double) = TrajectoryActionBuilderEx(builder.waitSeconds(t))
    /**
     * Waits [t] duration.
     */
    fun wait(t: Duration) = TrajectoryActionBuilderEx(builder.waitSeconds(t.toSeconds().value))

    /**
     * Schedules action [a] to execute in parallel starting at a displacement [ds] after the last trajectory segment.
     * The action start is clamped to the span of the current trajectory.
     *
     * Cannot be called without an applicable pending trajectory.
     */
    // TODO: Should calling this without an applicable trajectory implicitly begin an empty trajectory and execute the
    // action immediately?
    fun afterDisp(ds: Double, a: Action) = TrajectoryActionBuilderEx(builder.afterDisp(ds, a))

    /**
     * Schedules action [a] to execute in parallel starting at a displacement [ds] after the last trajectory segment.
     * The action start is clamped to the span of the current trajectory.
     *
     * Cannot be called without an applicable pending trajectory.
     */
    // TODO: Should calling this without an applicable trajectory implicitly begin an empty trajectory and execute the
    // action immediately?
    fun afterDisp(ds: Distance, a: Action) = TrajectoryActionBuilderEx(builder.afterDisp(ds.toInches().value, a))

    fun afterDisp(ds: Double, f: InstantFunction) = TrajectoryActionBuilderEx(builder.afterDisp(ds, f))

    fun afterDisp(ds: Distance, f: InstantFunction) = TrajectoryActionBuilderEx(builder.afterDisp(ds.toInches().value, f))

    /**
     * Schedules action [a] to execute in parallel starting [dt] seconds after the last trajectory segment, turn, or
     * other action.
     */
    fun afterTime(dt: Double, a: Action) = TrajectoryActionBuilderEx(builder.afterTime(dt, a))
    /**
     * Schedules action [a] to execute in parallel starting [dt] duration after the last trajectory segment, turn, or
     * other action.
     */
    fun afterTime(dt: Duration, a: Action) = TrajectoryActionBuilderEx(builder.afterTime(dt.toSeconds().value, a))
    fun afterTime(dt: Double, f: InstantFunction) = TrajectoryActionBuilderEx(builder.afterTime(dt, f))
    fun afterTime(dt: Duration, f: InstantFunction) = TrajectoryActionBuilderEx(builder.afterTime(dt.toSeconds().value, f))

    fun setTangent(r: Rotation2d) = TrajectoryActionBuilderEx(builder.setTangent(r))
    fun setTangent(r: Double) = TrajectoryActionBuilderEx(builder.setTangent(r))
    fun setTangent(r: Angle) = TrajectoryActionBuilderEx(builder.setTangent(r.toRadians().value))
    fun setReversed(reversed: Boolean) = TrajectoryActionBuilderEx(builder.setReversed(reversed))

    @JvmOverloads
    fun turn(angle: Double, turnConstraintsOverride: TurnConstraints? = null) = TrajectoryActionBuilderEx(builder.turn(angle, turnConstraintsOverride))
    @JvmOverloads
    fun turn(angle: Angle, turnConstraintsOverride: TurnConstraints? = null) = TrajectoryActionBuilderEx(builder.turn(angle.toRadians().value, turnConstraintsOverride))
    @JvmOverloads
    fun turnTo(heading: Rotation2d, turnConstraintsOverride: TurnConstraints? = null) = TrajectoryActionBuilderEx(builder.turnTo(heading, turnConstraintsOverride))
    @JvmOverloads
    fun turnTo(heading: Double, turnConstraintsOverride: TurnConstraints? = null) = TrajectoryActionBuilderEx(builder.turnTo(heading, turnConstraintsOverride))
    @JvmOverloads
    fun turnTo(heading: Angle, turnConstraintsOverride: TurnConstraints? = null) = TrajectoryActionBuilderEx(builder.turnTo(heading.toRadians().value, turnConstraintsOverride))

    @JvmOverloads
    fun lineToX(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.lineToX(posX, velConstraintOverride, accelConstraintOverride))

    @JvmOverloads
    fun lineToX(
        posX: Distance,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.lineToX(posX.toInches().value, velConstraintOverride, accelConstraintOverride))

    @JvmOverloads
    fun lineToXConstantHeading(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.lineToXConstantHeading(posX, velConstraintOverride, accelConstraintOverride))

    @JvmOverloads
    fun lineToXConstantHeading(
        posX: Distance,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.lineToXConstantHeading(posX.toInches().value, velConstraintOverride, accelConstraintOverride))

    @JvmOverloads
    fun lineToXLinearHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.lineToXLinearHeading(posX, heading, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun lineToXLinearHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.lineToXLinearHeading(posX, heading, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun lineToXLinearHeading(
        posX: Distance,
        heading: Angle,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.lineToXLinearHeading(posX.toInches().value, heading.toRadians().value, velConstraintOverride, accelConstraintOverride))

    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.lineToXSplineHeading(posX, heading, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.lineToXSplineHeading(posX, heading, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Distance,
        heading: Angle,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.lineToXSplineHeading(posX.toInches().value, heading.toRadians().value, velConstraintOverride, accelConstraintOverride))

    @JvmOverloads
    fun lineToY(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.lineToY(posY, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun lineToY(
        posY: Distance,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.lineToY(posY.toInches().value, velConstraintOverride, accelConstraintOverride))

    @JvmOverloads
    fun lineToYConstantHeading(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.lineToYConstantHeading(posY, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun lineToYConstantHeading(
        posY: Distance,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.lineToYConstantHeading(posY.toInches().value, velConstraintOverride, accelConstraintOverride))

    @JvmOverloads
    fun lineToYLinearHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.lineToYLinearHeading(posY, heading, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun lineToYLinearHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.lineToYLinearHeading(posY, heading, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun lineToYLinearHeading(
        posY: Distance,
        heading: Angle,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.lineToYLinearHeading(posY.toInches().value, heading.toRadians().value, velConstraintOverride, accelConstraintOverride))

    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.lineToYSplineHeading(posY, heading, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.lineToYSplineHeading(posY, heading, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Distance,
        heading: Angle,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.lineToYSplineHeading(posY.toInches().value, heading.toRadians().value, velConstraintOverride, accelConstraintOverride))

    @JvmOverloads
    fun strafeTo(
        pos: Vector2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.strafeTo(pos, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun <T: Distance, U: Distance> strafeTo(
        pos: Distance2d<T, U>,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.strafeTo(pos.toVector(), velConstraintOverride, accelConstraintOverride))

    @JvmOverloads
    fun strafeToConstantHeading(
        pos: Vector2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.strafeToConstantHeading(pos, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun <T: Distance, U: Distance> strafeToConstantHeading(
        pos: Distance2d<T, U>,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.strafeToConstantHeading(pos.toVector(), velConstraintOverride, accelConstraintOverride))

    @JvmOverloads
    fun strafeToLinearHeading(
        pos: Vector2d,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.strafeToLinearHeading(pos, heading, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun strafeToLinearHeading(
        pos: Vector2d,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.strafeToLinearHeading(pos, heading, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun <T: Distance, U: Distance> strafeToLinearHeading(
        pos: Distance2d<T, U>,
        heading: Angle,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.strafeToLinearHeading(pos.toVector(), heading.toRadians().value, velConstraintOverride, accelConstraintOverride))

    @JvmOverloads
    fun strafeToSplineHeading(
        pos: Vector2d,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.strafeToSplineHeading(pos, heading, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun strafeToSplineHeading(
        pos: Vector2d,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.strafeToSplineHeading(pos, heading, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun <T: Distance, U: Distance> strafeToSplineHeading(
        pos: Distance2d<T, U>,
        heading: Angle,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.strafeToSplineHeading(pos.toVector(), heading.toRadians().value, velConstraintOverride, accelConstraintOverride))

    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.splineTo(pos, tangent, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.splineTo(pos, tangent, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun <T: Distance, U: Distance> splineTo(
        pos: Distance2d<T, U>,
        tangent: Angle,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.splineTo(pos.toVector(), tangent.toRadians().value, velConstraintOverride, accelConstraintOverride))

    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.splineToConstantHeading(pos, tangent, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.splineToConstantHeading(pos, tangent, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun <T: Distance, U: Distance> splineToConstantHeading(
        pos: Distance2d<T, U>,
        tangent: Angle,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.splineToConstantHeading(pos.toVector(), tangent.toRadians().value, velConstraintOverride, accelConstraintOverride))

    @JvmOverloads
    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.splineToLinearHeading(pose, tangent, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.splineToLinearHeading(pose, tangent, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun <T: Distance, U: Distance, V: Angle> splineToLinearHeading(
        pose: Pose<T, U, V>,
        tangent: Angle,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.splineToLinearHeading(pose.toPose2d(), tangent.toRadians().value, velConstraintOverride, accelConstraintOverride))

    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.splineToSplineHeading(pose, tangent, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.splineToSplineHeading(pose, tangent, velConstraintOverride, accelConstraintOverride))
    @JvmOverloads
    fun <T: Distance, U: Distance, V: Angle> splineToSplineHeading(
        pose: Pose<T, U, V>,
        tangent: Angle,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilderEx(builder.splineToSplineHeading(pose.toPose2d(), tangent.toRadians().value, velConstraintOverride, accelConstraintOverride))

    /**
     * Creates a new builder with the same settings at the current pose, tangent.
     */
    fun fresh() = TrajectoryActionBuilderEx(builder.fresh())

    fun build() = builder.build()
}