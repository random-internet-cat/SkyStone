package org.firstinspires.ftc.teamcode.util

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