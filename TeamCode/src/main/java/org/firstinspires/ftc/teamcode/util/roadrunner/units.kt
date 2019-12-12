@file:Suppress("NOTHING_TO_INLINE", "OVERRIDE_BY_INLINE")

package org.firstinspires.ftc.teamcode.util.roadrunner

import org.firstinspires.ftc.teamcode.util.units.*

typealias RRAngle = Radians
typealias RRAnglePoint = RadiansPoint
typealias RRDuration = Seconds
typealias RRDistance = Meters
typealias RRVelocity = MetersPerSecond
typealias RRAcceleration = MetersPerSecondSquared
typealias RRJerk = MetersPerSecondCubed
typealias RRDistancePerRev = MetersPerRev
typealias RRAngularVelocity = RadiansPerSecond
typealias RRAngularAcceleration = RadiansPerSecondSquared
typealias RRAngularJerk = RadiansPerSecondCubed

inline fun RRAngle.roadrunner(): RRAngle = this
fun Angle.roadrunner(): RRAngle = Radians(this)

inline fun RRAnglePoint.roadrunner(): RRAnglePoint = this
fun AnglePoint.roadrunner(): RRAnglePoint = RadiansPoint(this)

inline fun RRDuration.roadrunner(): RRDuration = this
fun Duration.roadrunner(): RRDuration = Seconds(this)

inline fun RRDistance.roadrunner(): RRDistance = this
fun Distance.roadrunner(): RRDistance = Meters(this)

inline fun RRVelocity.roadrunner(): RRVelocity = this
fun Velocity.roadrunner(): RRVelocity = MetersPerSecond(this)

inline fun RRAcceleration.roadrunner(): RRAcceleration = this
fun Acceleration.roadrunner(): RRAcceleration = MetersPerSecondSquared(this)

inline fun RRJerk.roadrunner(): RRJerk = this
fun Jerk.roadrunner(): RRJerk = MetersPerSecondCubed(this)

inline fun RRDistancePerRev.roadrunner(): RRDistancePerRev = this
fun DistancePerRev.roadrunner(): RRDistancePerRev = MetersPerRev(this)

inline fun RRAngularVelocity.roadrunner(): RRAngularVelocity = this
fun AngularVelocity.roadrunner(): RRAngularVelocity = RadiansPerSecond(this)

inline fun RRAngularAcceleration.roadrunner(): RRAngularAcceleration = this
fun AngularAcceleration.roadrunner(): RRAngularAcceleration = RadiansPerSecondSquared(this)

inline fun RRAngularJerk.roadrunner(): RRAngularJerk = this
fun AngularJerk.roadrunner(): RRAngularJerk = RadiansPerSecondCubed(this)