package org.firstinspires.ftc.teamcode.hardware.drive.constants

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.MecanumDriveConfig
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.maxVelocity
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.ticksPerSecond
import org.firstinspires.ftc.teamcode.hardware.drive.motors.YellowJacket5202_0002_0019
import org.firstinspires.ftc.teamcode.util.*
import org.firstinspires.ftc.teamcode.util.roadrunner.*
import org.firstinspires.ftc.teamcode.util.units.*
import org.firstinspires.ftc.teamcode.util.units.MetersPerSecond

@Config
object MarkIDriveConstants : MecanumDriveConfig {
    private val MOTOR_TYPE = YellowJacket5202_0002_0019::class.java
    private val MOTOR_CONFIG = MotorConfigurationType.getMotorType(MOTOR_TYPE)
    private val TICKS_PER_REV = EncoderTicksPerRev(MOTOR_CONFIG.ticksPerRev)
    private val MAX_RPM = RevolutionsPerMinute(MOTOR_CONFIG.maxRPM)
    private val MAX_ACHIEVABLE_RPM = MAX_RPM * MOTOR_CONFIG.achieveableMaxRPMFraction

    @JvmField
    public var GEAR_RATIO: Double = 1.0

    @JvmField
    public var _WHEEL_RADIUS_IN: Double = 2.0

    fun wheelRadius(): RRDistance {
        return Inches(_WHEEL_RADIUS_IN).roadrunner()
    }

    @JvmField
    public var _TRACK_WIDTH_IN: Double = 19.0

    override fun trackWidth(): RRDistance {
        return Inches(_TRACK_WIDTH_IN).roadrunner()
    }

    @JvmField
    public var _WHEEL_BASE_IN: Double = 13.22

    override fun wheelBase(): RRDistance {
        return Inches(_WHEEL_BASE_IN).roadrunner()
    }

    private val TRANSLATIONAL_PID = PIDCoefficients(0.0, 0.0, 0.0)

    override fun translationalPID(): PIDCoefficients {
        return TRANSLATIONAL_PID
    }

    private val HEADING_PID = PIDCoefficients(0.0, 0.0, 0.0)

    override fun headingPID(): PIDCoefficients {
        return HEADING_PID
    }

    fun wheelCircumference() = (wheelRadius() * TWO_PI).roadrunner()
    fun distancePerRev() = ((wheelCircumference() * GEAR_RATIO) / Revolutions(1.0)).roadrunner()

    private val BASE_CONSTRAINTS = DriveConstraints(Feet(5) / Seconds(1), Inches(20) / Seconds(1) / Seconds(1), RevolutionsPerSecond(0.5), DegreesPerSecondSquared(180))

    override fun baseConstraints(): DriveConstraints {
        return BASE_CONSTRAINTS
    }

    private val CHARACTERIZATION = DcMotorCharacterization.forBuiltinPID(kV = 1 / (maxVelocity().roadrunner().raw))

    override fun characterization(): DcMotorCharacterization {
        return CHARACTERIZATION
    }

    // Per https://github.com/acmerobotics/road-runner-quickstart/commit/2c4968515de06f094afc0d51fb3c564c3ab32b3d
    override fun motorKF(): Double {
        return 32767.toDouble() / ticksPerSecond().raw.toDouble()
    }

    override fun wheelAngularVelocityToLinear(angular: RRAngularVelocity): RRVelocity {
        val revPerSec = RevolutionsPerSecond(angular)
        return MetersPerSecond(distancePerRev() * revPerSec).roadrunner()
    }

    override fun encoderTicksToDistance(ticks: EncoderTicks): RRDistance {
        return (distancePerRev() * (ticks / TICKS_PER_REV)).roadrunner()
    }

    override fun distanceToEncoderTicks(distance: RRDistance): EncoderTicks {
        return (distance / distancePerRev()) * TICKS_PER_REV
    }

    override fun ticksPerRev() = TICKS_PER_REV

    override fun maxWheelRPM() = MAX_ACHIEVABLE_RPM.roadrunner()
}