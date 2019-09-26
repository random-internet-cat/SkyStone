package org.firstinspires.ftc.teamcode.util.roadrunner

import org.firstinspires.ftc.teamcode.util.units.*

typealias RRAngle = Radians
typealias RRAnglePoint = RadiansPoint
typealias RRTime = Seconds
typealias RRDistance = Meters
typealias RRVelocity = MetersPerSecond
typealias RRAcceleration = MetersPerSecondSquared
typealias RRJerk = MetersPerSecondCubed
typealias RRAngularVelocity = RadiansPerSecond
typealias RRAngularAcceleration = RadiansPerSecondSquared
typealias RRAngularJerk = RadiansPerSecondCubed

inline fun RRAngle.roadrunner(): RRAngle = this
fun Angle.roadrunner(): RRAngle = Radians(this)

inline fun RRAnglePoint.roadrunner(): RRAnglePoint = this
fun AnglePoint.roadrunner(): RRAnglePoint = RadiansPoint(this)

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