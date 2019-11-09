package org.firstinspires.ftc.teamcode.util.roadrunner

import com.acmerobotics.roadrunner.profile.MotionState
import org.firstinspires.ftc.teamcode.util.units.*

fun MotionState(position: RRDistance, vel: RRVelocity, accel: RRAcceleration, jerk: RRJerk) = MotionState(position.roadrunner().raw, vel.roadrunner().raw, accel.roadrunner().raw, jerk.roadrunner().raw)
fun MotionState(position: Distance, vel: Velocity, accel: Acceleration, jerk: Jerk) = MotionState(position.roadrunner(), vel.roadrunner(), accel.roadrunner(), jerk.roadrunner())

fun MotionState(position: RRAnglePoint, vel: RRAngularVelocity, accel: RRAngularAcceleration, jerk: RRAngularJerk) = MotionState(position.roadrunner().raw, vel.roadrunner().raw, accel.roadrunner().raw, jerk.roadrunner().raw)
fun MotionState(position: AnglePoint, vel: AngularVelocity, accel: AngularAcceleration, jerk: AngularJerk) = MotionState(position.roadrunner(), vel.roadrunner(), accel.roadrunner(), jerk.roadrunner())
