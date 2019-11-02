package org.firstinspires.ftc.teamcode.util.roadrunner

import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import org.firstinspires.ftc.teamcode.util.units.*

fun MotionProfileGenerator.generateSimpleMotionProfile(
    start: MotionState,
    goal: MotionState,
    maxVel: RRVelocity,
    maxAccel: RRAcceleration,
    maxJerk: RRJerk = RRJerk(0.0),
    overshoot: Boolean = false
) = generateSimpleMotionProfile(start, goal, maxVel.roadrunner().raw, maxAccel.roadrunner().raw, maxJerk.roadrunner().raw, overshoot)

fun MotionProfileGenerator.generateSimpleMotionProfile(
    start: MotionState,
    goal: MotionState,
    maxVel: Velocity,
    maxAccel: Acceleration,
    maxJerk: Jerk = Jerk.zero(),
    overshoot: Boolean = false
) = generateSimpleMotionProfile(start, goal, maxVel.roadrunner(), maxAccel.roadrunner(), maxJerk.roadrunner(), overshoot)

fun MotionProfileGenerator.generateSimpleMotionProfile(
    start: MotionState,
    goal: MotionState,
    maxVel: RRAngularVelocity,
    maxAccel: RRAngularAcceleration,
    maxJerk: RRAngularJerk = RRAngularJerk(0.0),
    overshoot: Boolean = false
) = generateSimpleMotionProfile(start, goal, maxVel.roadrunner().raw, maxAccel.roadrunner().raw, maxJerk.roadrunner().raw, overshoot)

fun MotionProfileGenerator.generateSimpleMotionProfile(
    start: MotionState,
    goal: MotionState,
    maxVel: AngularVelocity,
    maxAccel: AngularAcceleration,
    maxJerk: AngularJerk = AngularJerk.zero(),
    overshoot: Boolean = false
) = generateSimpleMotionProfile(start, goal, maxVel.roadrunner(), maxAccel.roadrunner(), maxJerk.roadrunner(), overshoot)