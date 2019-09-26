package org.firstinspires.ftc.teamcode.util.roadrunner

import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.TankConstraints
import org.firstinspires.ftc.teamcode.util.units.*

inline fun DriveConstraints(maxVel: RRVelocity, maxAccel: RRAcceleration, maxJerk: RRJerk, maxAngVel: RRAngularVelocity, maxAngAccel: RRAngularAcceleration, maxAngJerk: RRAngularJerk) = DriveConstraints(maxVel.roadrunner().raw, maxAccel.roadrunner().raw, maxJerk.roadrunner().raw, maxAngVel.roadrunner().raw, maxAngAccel.roadrunner().raw, maxAngJerk.roadrunner().raw)
fun DriveConstraints(maxVel: Velocity, maxAccel: Acceleration, maxJerk: Jerk, maxAngVel: AngularVelocity, maxAngAccel: AngularAcceleration, maxAngJerk: AngularJerk) = DriveConstraints(maxVel.roadrunner(), maxAccel.roadrunner(), maxJerk.roadrunner(), maxAngVel.roadrunner(), maxAngAccel.roadrunner(), maxAngJerk.roadrunner())

inline fun DriveConstraints(maxVel: RRVelocity, maxAccel: RRAcceleration, maxAngVel: RRAngularVelocity, maxAngAccel: RRAngularAcceleration) = DriveConstraints(maxVel.roadrunner().raw, maxAccel.roadrunner().raw, 0.0, maxAngVel.roadrunner().raw, maxAngAccel.roadrunner().raw, 0.0)
fun DriveConstraints(maxVel: Velocity, maxAccel: Acceleration, maxAngVel: AngularVelocity, maxAngAccel: AngularAcceleration) = DriveConstraints(maxVel.roadrunner(), maxAccel.roadrunner(), maxAngVel.roadrunner(), maxAngAccel.roadrunner())

fun TankConstraints(baseConstraints: DriveConstraints, trackWidth: RRDistance) = TankConstraints(baseConstraints, trackWidth.roadrunner().raw)
fun TankConstraints(baseConstraints: DriveConstraints, trackWidth: Distance) = TankConstraints(baseConstraints, trackWidth.roadrunner())