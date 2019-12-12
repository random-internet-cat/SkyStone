package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.util.roadrunner.PositionVector
import org.firstinspires.ftc.teamcode.util.roadrunner.roadrunner
import org.firstinspires.ftc.teamcode.util.units.*

object RRUnits {
    @JvmStatic
    fun inches(inches: Double) = Inches(inches).roadrunner().raw

    @JvmStatic
    fun inchesVector(x: Double, y: Double) = Vector2d(inches(x), inches(y))

    @JvmStatic
    fun feet(feet: Double) = Feet(feet).roadrunner().raw

    @JvmStatic
    fun feetVector(x: Double, y: Double) = Vector2d(feet(x), feet(y))

    @JvmStatic
    fun meters(meters: Double) = Meters(meters).roadrunner().raw

    @JvmStatic
    fun metersVector(x: Double, y: Double) = Vector2d(meters(x), meters(y))

    @JvmStatic
    fun zeroDistance() = Distance.zero().roadrunner().raw

    @JvmStatic
    fun radians(radians: Double) = Radians(radians).roadrunner().raw

    @JvmStatic
    fun degrees(degrees: Double) = Degrees(degrees).roadrunner().raw

    @JvmStatic
    fun zeroHeading() = AnglePoint.zero().roadrunner().raw

    @JvmStatic
    fun rr(angle: Angle) = angle.roadrunner().raw

    @JvmStatic
    fun rr(anglePoint: AnglePoint) = anglePoint.roadrunner().raw

    @JvmStatic
    fun rr(duration: Duration) = duration.roadrunner().raw

    @JvmStatic
    fun rr(distance: Distance) = distance.roadrunner().raw

    @JvmStatic
    fun rr(velocity: Velocity) = velocity.roadrunner().raw

    @JvmStatic
    fun rr(acceleration: Acceleration) = acceleration.roadrunner().raw

    @JvmStatic
    fun rr(jerk: Jerk) = jerk.roadrunner().raw

    @JvmStatic
    fun rr(distancePerRev: DistancePerRev) = distancePerRev.roadrunner().raw

    @JvmStatic
    fun rr(angularVelocity: AngularVelocity) = angularVelocity.roadrunner().raw

    @JvmStatic
    fun rr(angularAcceleration: AngularAcceleration) = angularAcceleration.roadrunner().raw

    @JvmStatic
    fun rr(angularJerk: AngularJerk) = angularJerk.roadrunner().raw
}