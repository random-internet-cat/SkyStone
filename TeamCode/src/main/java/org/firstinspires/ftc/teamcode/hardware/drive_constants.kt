package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.teamcode.util.*

object DriveConstants {
    private val MOTOR_TYPE = YellowJacket5202_0002_0019::class.java
    private val MOTOR_CONFIG = MotorConfigurationType.getMotorType(MOTOR_TYPE)
    private val TICKS_PER_REV = MOTOR_CONFIG.ticksPerRev
    private val MAX_RPM = RevolutionsPerMinute(MOTOR_CONFIG.maxRPM)

    // Necessary for tuning via FTC Dashboard
    public var _WHEEL_RADIUS_IN: Double = 2.0
    public var _TRACK_WIDTH_IN: Double = 1.0

    public val WHEEL_RADIUS: RRDistance get() = Inches(_WHEEL_RADIUS_IN).roadrunner()
    public val TRACK_WIDTH: RRDistance get() = Inches(_TRACK_WIDTH_IN).roadrunner()

    public var GEAR_RATIO: Double = 1.0

    public val WHEEL_CIRCUMFERENCE: RRDistance get() = (WHEEL_RADIUS * TWO_PI).roadrunner()
    public val DISTANCE_PER_REVOLUTION: RRDistance get() = (WHEEL_CIRCUMFERENCE * GEAR_RATIO).roadrunner()

    public var kV: Double = 1 / (rpmToVelocity(MAX_RPM).roadrunner().raw)
    public var kA: Double = 0.0
    public var kStatic: Double = 0.0

    fun rpmToVelocity(revolutionSpeed: RevolutionSpeed): RRVelocity {
        val revPerSec = RevolutionsPerSecond(revolutionSpeed)
        return MetersPerSecond(Meters(DISTANCE_PER_REVOLUTION).raw * revPerSec.raw).roadrunner()
    }

    fun encoderTicksToDistance(ticks: Int): RRDistance {
        return (DISTANCE_PER_REVOLUTION * ticks / TICKS_PER_REV).roadrunner()
    }
}