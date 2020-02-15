package org.firstinspires.ftc.teamcode.hardware.drive.constants

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.*
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
    public var _TRACK_WIDTH_IN: Double = 15.82

    override fun trackWidth(): RRDistance {
        return Inches(_TRACK_WIDTH_IN).roadrunner()
    }

    @JvmField
    public var _WHEEL_BASE_IN: Double = 13.22

    override fun wheelBase(): RRDistance {
        return Inches(_WHEEL_BASE_IN).roadrunner()
    }

    private val TRANSLATIONAL_PID = PIDCoefficients(1.5, 0.0, 0.0)

    override fun translationalPID(): PIDCoefficients {
        return TRANSLATIONAL_PID
    }

    private val HEADING_PID = PIDCoefficients(1.0, 0.0, 0.0)

    override fun headingPID(): PIDCoefficients {
        return HEADING_PID
    }

    fun wheelCircumference() = (wheelRadius() * TWO_PI).roadrunner()
    fun distancePerRev() = ((wheelCircumference() * GEAR_RATIO) / Revolutions(1.0)).roadrunner()

    private val BASE_CONSTRAINTS = DriveConstraints(Feet(5) / Seconds(1), Inches(20) / Seconds(1) / Seconds(1), RevolutionsPerSecond(0.5), DegreesPerSecondSquared(180))

    override fun baseConstraints(): DriveConstraints {
        return BASE_CONSTRAINTS
    }

    // Per https://github.com/acmerobotics/road-runner-quickstart/commit/2c4968515de06f094afc0d51fb3c564c3ab32b3d
    private fun pidMotorKF(): Double {
        return 32767.toDouble() / ticksPerSecond().raw.toDouble()
    }

    private fun pidKV(): Double {
        return 1 / (maxVelocity().roadrunner().raw)
    }

    private sealed class PIDOrFeedforward {
        data class UsePID(val coefficients: PIDCoefficients) : PIDOrFeedforward()
        data class UseFeedforward(val characterization: DcMotorCharacterization) : PIDOrFeedforward()
    }

    private val PID_OR_FEEDFORWARD: PIDOrFeedforward = PIDOrFeedforward.UsePID(PIDCoefficients(28.0, 12.0, 7.0)) //TODO: EDIT THESE CONSTANTS
    // Option: (P: 1.17, I: .117, D: 0)
    // Other option: (P: 33, I: 0, D: 7)

    override fun pidOrFeedforward(): MecanumPIDOrFeedForward {
        return when (PID_OR_FEEDFORWARD) {
            is PIDOrFeedforward.UsePID -> MecanumUsePID(PID_OR_FEEDFORWARD.coefficients, kV = pidKV(), motorKF = pidMotorKF())
            is PIDOrFeedforward.UseFeedforward -> MecanumUseFeedforward(PID_OR_FEEDFORWARD.characterization)
        }
    }

    override fun wheelAngularVelocityToLinear(angular: AngularVelocity): RRVelocity {
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