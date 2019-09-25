package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints

typealias RRAbsoluteAngle = AbsoluteRadians
typealias RRRelativeAngle = RelativeRadians
typealias RRTime = Seconds
typealias RRDistance = Meters
typealias RRVelocity = MetersPerSecond
typealias RRAcceleration = MetersPerSecondSquared
typealias RRJerk = MetersPerSecondCubed
typealias RRAngularVelocity = RadiansPerSecond
typealias RRAngularAcceleration = RadiansPerSecondSquared
typealias RRAngularJerk = RadiansPerSecondCubed

inline fun RRAbsoluteAngle.roadrunner(): RRAbsoluteAngle = this
fun AbsoluteAngle.roadrunner(): RRAbsoluteAngle = AbsoluteRadians(this)

inline fun RRRelativeAngle.roadrunner(): RRRelativeAngle = this
fun RelativeAngle.roadrunner(): RRRelativeAngle = RelativeRadians(this)

inline fun RRTime.roadrunner(): RRTime = this
fun Time.roadrunner(): RRTime = Seconds(this)

inline fun RRDistance.roadrunner(): RRDistance = this
fun Distance.roadrunner(): RRDistance = Meters(this)

inline fun RRVelocity.roadrunner(): RRVelocity = this
fun Velocity.roadrunner(): RRVelocity = MetersPerSecond(this)

inline fun RRAcceleration.roadrunner(): RRAcceleration = this
fun Acceleration.roadrunner(): RRAcceleration = MetersPerSecondSquared(this)

inline fun RRJerk.roadrunner(): RRJerk = this
fun Jerk.roadrunner(): RRJerk = MetersPerSecondCubed(this)

inline fun RRAngularVelocity.roadrunner(): RRAngularVelocity = this
fun AngularVelocity.roadrunner(): RRAngularVelocity = RadiansPerSecond(this)

inline fun RRAngularAcceleration.roadrunner(): RRAngularAcceleration = this
fun AngularAcceleration.roadrunner(): RRAngularAcceleration = RadiansPerSecondSquared(this)

inline fun RRAngularJerk.roadrunner(): RRAngularJerk = this
fun AngularJerk.roadrunner(): RRAngularJerk = RadiansPerSecondCubed(this)

inline fun DriveConstraints(maxVel: RRVelocity, maxAccel: RRAcceleration, maxJerk: RRJerk, maxAngVel: RRAngularVelocity, maxAngAccel: RRAngularAcceleration, maxAngJerk: RRAngularJerk) = DriveConstraints(maxVel.roadrunner().raw, maxAccel.roadrunner().raw, maxJerk.roadrunner().raw, maxAngVel.roadrunner().raw, maxAngAccel.roadrunner().raw, maxAngJerk.roadrunner().raw)
fun DriveConstraints(maxVel: Velocity, maxAccel: Acceleration, maxJerk: Jerk, maxAngVel: AngularVelocity, maxAngAccel: AngularAcceleration, maxAngJerk: AngularJerk) = DriveConstraints(maxVel.roadrunner(), maxAccel.roadrunner(), maxJerk.roadrunner(), maxAngVel.roadrunner(), maxAngAccel.roadrunner(), maxAngJerk.roadrunner())

inline fun DriveConstraints(maxVel: RRVelocity, maxAccel: RRAcceleration, maxAngVel: RRAngularVelocity, maxAngAccel: RRAngularAcceleration) = DriveConstraints(maxVel.roadrunner().raw, maxAccel.roadrunner().raw, 0.0, maxAngVel.roadrunner().raw, maxAngAccel.roadrunner().raw, 0.0)
fun DriveConstraints(maxVel: Velocity, maxAccel: Acceleration, maxAngVel: AngularVelocity, maxAngAccel: AngularAcceleration) = DriveConstraints(maxVel.roadrunner(), maxAccel.roadrunner(), maxAngVel.roadrunner(), maxAngAccel.roadrunner())