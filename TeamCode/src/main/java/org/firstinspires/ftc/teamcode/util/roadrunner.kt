package org.firstinspires.ftc.teamcode.util

typealias RRAbsoluteAngle = AbsoluteRadians
typealias RRRelativeAngle = RelativeRadians
typealias RRTime = Seconds
typealias RRDistance = Meters
typealias RRVelocity = MetersPerSecond

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