package org.firstinspires.ftc.teamcode.hardware.drive.tank

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.teamcode.hardware.drive.motors.YellowJacket5202_0002_0019
import org.firstinspires.ftc.teamcode.util.*
import org.firstinspires.ftc.teamcode.util.roadrunner.*
import org.firstinspires.ftc.teamcode.util.units.*
import org.firstinspires.ftc.teamcode.util.units.MetersPerSecond

@Config
object TankDriveConstants {
    private val MOTOR_TYPE = YellowJacket5202_0002_0019::class.java
    private val MOTOR_CONFIG = MotorConfigurationType.getMotorType(MOTOR_TYPE)
    private val TICKS_PER_REV = MOTOR_CONFIG.ticksPerRev
    private val MAX_RPM = RevolutionsPerMinute(MOTOR_CONFIG.maxRPM)

    // Necessary for tuning via FTC Dashboard
    @JvmField
    public var _WHEEL_RADIUS_IN: Double = 2.0

    @JvmField
    public var _TRACK_WIDTH_IN: Double = 1.0

    public val WHEEL_RADIUS: RRDistance get() = Inches(_WHEEL_RADIUS_IN).roadrunner()
    public val TRACK_WIDTH: RRDistance get() = Inches(_TRACK_WIDTH_IN).roadrunner()

    @JvmField
    public var GEAR_RATIO: Double = 1.0

    public val WHEEL_CIRCUMFERENCE: RRDistance get() = (WHEEL_RADIUS * TWO_PI).roadrunner()
    public val DISTANCE_PER_REVOLUTION: RRDistance get() = (WHEEL_CIRCUMFERENCE * GEAR_RATIO).roadrunner()

    public val FEEDFORWARD = DcMotorFeedforward(kV = 1 / (rpmToVelocity(MAX_RPM).roadrunner().raw), kA = 0.0, kStatic = 0.0)

    public val AXIAL_PID = PIDCoefficients(0.0, 0.0, .0)
    public val HEADING_PID = PIDCoefficients(0.0, 0.0, .0)
    public val CROSS_TRACK_PID = PIDCoefficients(0.0, 0.0, .0)

    @JvmField
    public val BASE_CONSTRAINTS = DriveConstraints(Inches(30) / Seconds(1), Inches(30) / Seconds(1) / Seconds(1), DegreesPerSecond(180), DegreesPerSecondSquared(180.0))

    fun rpmToVelocity(revolutionSpeed: AngularVelocity): RRVelocity {
        val revPerSec = RevolutionsPerSecond(revolutionSpeed)
        return MetersPerSecond(Meters(DISTANCE_PER_REVOLUTION).raw * revPerSec.raw)
            .roadrunner()
    }

    fun encoderTicksToDistance(ticks: Int): RRDistance {
        return (DISTANCE_PER_REVOLUTION * ticks / TICKS_PER_REV).roadrunner()
    }
}