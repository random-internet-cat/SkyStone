package org.firstinspires.ftc.teamcode.util.roadrunner

import org.firstinspires.ftc.teamcode.util.units.AnglePoint
import com.acmerobotics.roadrunner.profile.MotionState
import org.firstinspires.ftc.teamcode.util.units.AngularAcceleration
import org.firstinspires.ftc.teamcode.util.units.AngularJerk
import org.firstinspires.ftc.teamcode.util.units.AngularVelocity

fun MotionState(position: RRAnglePoint, vel: RRAngularVelocity, accel: RRAngularAcceleration, jerk: RRAngularJerk) = MotionState(position.roadrunner().raw, vel.roadrunner().raw, accel.roadrunner().raw, jerk.roadrunner().raw)
fun MotionState(position: AnglePoint, vel: AngularVelocity, accel: AngularAcceleration, jerk: AngularJerk) = MotionState(position.roadrunner(), vel.roadrunner(), accel.roadrunner(), jerk.roadrunner())
