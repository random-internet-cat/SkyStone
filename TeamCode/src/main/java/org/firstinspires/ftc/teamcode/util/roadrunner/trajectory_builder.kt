package org.firstinspires.ftc.teamcode.util.roadrunner

import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder
import org.firstinspires.ftc.teamcode.util.units.Distance
import org.firstinspires.ftc.teamcode.util.units.Duration

fun BaseTrajectoryBuilder.lineTo(pos: PositionVector) = lineTo(pos.roadrunner())
fun BaseTrajectoryBuilder.lineTo(pos: PositionVector, interpolator: HeadingInterpolator) = lineTo(pos.roadrunner(), interpolator)

fun BaseTrajectoryBuilder.strafeTo(pos: PositionVector) = strafeTo(pos.roadrunner())

fun BaseTrajectoryBuilder.forward(distance: RRDistance) = forward(distance.roadrunner().raw)
fun BaseTrajectoryBuilder.forward(distance: Distance) = forward(distance.roadrunner())

fun BaseTrajectoryBuilder.back(distance: RRDistance) = back(distance.roadrunner().raw)
fun BaseTrajectoryBuilder.back(distance: Distance) = back(distance.roadrunner())

fun BaseTrajectoryBuilder.strafeLeft(distance: RRDistance) = strafeLeft(distance.roadrunner().raw)
fun BaseTrajectoryBuilder.strafeLeft(distance: Distance) = strafeLeft(distance.roadrunner())

fun BaseTrajectoryBuilder.strafeRight(distance: RRDistance) = strafeRight(distance.roadrunner().raw)
fun BaseTrajectoryBuilder.strafeRight(distance: Distance) = strafeRight(distance.roadrunner())

fun BaseTrajectoryBuilder.splineTo(pos: RobotPosition) = splineTo(pos.roadrunner())
fun BaseTrajectoryBuilder.splineTo(pos: RobotPosition, interpolator: HeadingInterpolator) = splineTo(pos.roadrunner(), interpolator)

fun BaseTrajectoryBuilder.addMarker(time: RRDuration, callback: () -> Unit) = addMarker(time.roadrunner().raw, callback)
fun BaseTrajectoryBuilder.addMarker(time: Duration, callback: () -> Unit) = addMarker(time.roadrunner(), callback)

fun BaseTrajectoryBuilder.addMarker(pos: PositionVector, callback: () -> Unit) = addMarker(pos.roadrunner(), callback)