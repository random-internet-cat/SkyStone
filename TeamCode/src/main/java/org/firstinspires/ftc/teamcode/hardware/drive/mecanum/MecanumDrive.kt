package org.firstinspires.ftc.teamcode.hardware.drive.mecanum

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDrivetrainConfig
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumPID
import org.firstinspires.ftc.teamcode.drive.mecanum.RRMecanumDriveBase
import org.firstinspires.ftc.teamcode.hardware.drive.*
import org.firstinspires.ftc.teamcode.util.*
import org.firstinspires.ftc.teamcode.util.roadrunner.*
import org.firstinspires.ftc.teamcode.util.units.*

typealias MecanumDriveMotor = TypedMotorEx

data class MecanumDrivetrain(val frontLeft: MecanumDriveMotor, val frontRight: MecanumDriveMotor, val backLeft: MecanumDriveMotor, val backRight: MecanumDriveMotor) {
    private val fourWheelValue by lazy {
        FourWheelDrivetrainEx(
            frontLeft = frontLeft.motor,
            frontRight = frontRight.motor,
            backLeft = backLeft.motor,
            backRight = backRight.motor
        )
    }

    fun toFourWheelDrivetrain() = fourWheelValue

    fun typedMotors() = listOf(frontLeft, frontRight, backLeft, backRight)
}

sealed class MecanumPIDOrFeedForward {
    abstract val characterization: DcMotorCharacterization
}

data class MecanumUsePID(val initialCoefficients: PIDCoefficients, private val kV: Double, val motorKF: Double) : MecanumPIDOrFeedForward() {
    override val characterization = DcMotorCharacterization.forBuiltinPID(kV = kV)
}

data class MecanumUseFeedforward(override val characterization: DcMotorCharacterization) : MecanumPIDOrFeedForward()

interface MecanumDriveConfig {
    fun trackWidth(): Distance
    fun wheelBase(): Distance

    fun translationalPID(): PIDCoefficients
    fun headingPID(): PIDCoefficients

    fun pidOrFeedforward(): MecanumPIDOrFeedForward

    fun baseConstraints(): DriveConstraints

    fun ticksPerRev(): EncoderTicksPerRev
    fun maxWheelRPM(): AngularVelocity

    fun encoderTicksToDistance(ticks: EncoderTicks): Distance
    fun distanceToEncoderTicks(distance: RRDistance): EncoderTicks

    fun wheelAngularVelocityToLinear(angular: AngularVelocity): Velocity
}

fun MecanumDriveConfig.maxVelocity() = RRVelocity(baseConstraints().maxVel)
fun MecanumDriveConfig.maxDriveRPM() = RRAngularVelocity(baseConstraints().maxAngVel)

fun MecanumDriveConfig.ticksPerSecond() = EncoderTicksPerSecond(RevolutionsPerSecond(maxWheelRPM()).raw * ticksPerRev().raw)

fun MecanumDriveConfig.encoderTicksToDistance(ticks: EncoderPosition) = encoderTicksToDistance(EncoderTicks(ticks.raw))

fun MecanumDriveConfig.characterization() = pidOrFeedforward().characterization

sealed class MecanumHeadingConfig {
    abstract val useExternalHeading: Boolean
    abstract fun currentHeading(): Heading
}

class BadHeadingAccessException : Exception("Invalid request for heading; perhaps a bug in roadrunner?")

object MecanumNoExternalHeading : MecanumHeadingConfig() {
    override val useExternalHeading: Boolean = false
    override fun currentHeading(): Nothing = throw BadHeadingAccessException()
}

data class MecanumUseHeadingProvider(private val headingProvider: HeadingProvider) : MecanumHeadingConfig() {
    override val useExternalHeading: Boolean = true
    override fun currentHeading(): Heading = headingProvider.currentHeading()
}

sealed class MecanumLocalizationConfig(val useWheelEncoders: Boolean) {
    abstract val useExternalHeading: Boolean
    abstract fun currentHeading(): Heading
}

class MecanumNoWheelEncoders(val localizer: ThreeTrackingWheelLocalizer) : MecanumLocalizationConfig(false) {
    override val useExternalHeading: Boolean = false
    override fun currentHeading(): Heading = throw BadHeadingAccessException()
}

class MecanumUseWheelEncoders(val headingConfig: MecanumHeadingConfig) : MecanumLocalizationConfig(true) {
    override val useExternalHeading: Boolean get() = headingConfig.useExternalHeading
    override fun currentHeading(): Heading = headingConfig.currentHeading()
}

class MecanumDrive(private val localizationConfig: MecanumLocalizationConfig, val config: MecanumDriveConfig, drivetrain: MecanumDrivetrain) : BaseDriveEx(drivetrain.toFourWheelDrivetrain()) {
    private val pidOrFeedForward = config.pidOrFeedforward()
    private val _onlyPID = pidOrFeedForward as? MecanumUsePID

    private val roadrunnerValue by lazy {
        object : RRMecanumDriveBase(MecanumDrivetrainConfig(trackWidth = config.trackWidth(), wheelBase = config.wheelBase()), MecanumPID(config.translationalPID(), config.headingPID()), pidOrFeedForward.characterization, config.baseConstraints()) {
            private fun positionOf(motor: TypedMotor) = config.encoderTicksToDistance(motor.encoderPosition())
            private fun velocityOf(motor: DcMotorEx): RRVelocity = (config.encoderTicksToDistance(EncoderTicks(motor.getVelocity().toInt())) / Seconds(1)).roadrunner()

            override fun getPIDCoefficients(runMode: DcMotor.RunMode): PIDCoefficients {
                requirePID()

                val coeffs = drivetrain.frontLeft.motor.getPIDFCoefficients(runMode)
                return PIDCoefficients(coeffs.p, coeffs.i, coeffs.d)
            }

            override fun setPIDCoefficients(runMode: DcMotor.RunMode, coefficients: PIDCoefficients) {
                val pidConfig = requirePID()
                val kF = pidConfig.motorKF

                forEachMotor {
                    setPIDF(runMode, coefficients.kP, coefficients.kI, coefficients.kD, kF)
                }
            }

            override val rawExternalHeading: Double get() {
                return RadiansPoint(localizationConfig.currentHeading()).raw
            }

            override var localizer: Localizer =
                when (localizationConfig) {
                    is MecanumNoWheelEncoders -> localizationConfig.localizer
                    is MecanumUseWheelEncoders -> MecanumLocalizer(this, useExternalHeading = localizationConfig.useExternalHeading)
                }

            private fun rrOrderedMotors() = drivetrain.run { listOf(frontLeft, backLeft, backRight, frontRight) }

            override fun getWheelVelocities(): List<Double> {
                requireEncoders()
                return rrOrderedMotors().map { velocityOf(it.motor).roadrunner().raw }
            }

            override fun getWheelPositions(): List<Double> {
                requireEncoders()
                return rrOrderedMotors().map { positionOf(it).roadrunner().raw }
            }

            override fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double) {
                drivetrain.frontLeft.setPower(frontLeft)
                drivetrain.backLeft.setPower(rearLeft)
                drivetrain.backRight.setPower(rearRight)
                drivetrain.frontRight.setPower(frontRight)
            }
        }
    }

    init {
        if (pidOrFeedForward is MecanumUsePID) roadrunner().setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidOrFeedForward.initialCoefficients)
    }

    private fun requirePID(): MecanumUsePID {
        check(_onlyPID != null)
        return _onlyPID
    }

    private fun requireEncoders() {
        check(localizationConfig.useWheelEncoders)
    }

    fun roadrunner() = roadrunnerValue

    fun update(updateDashboard: Boolean = true) = roadrunner().update(updateDashboard = updateDashboard)
}

fun MecanumDrive.mecanumDrive(x: RRVelocity, y: RRVelocity, turn: RRAngularVelocity) {
    roadrunner().setDriveSignal(DriveSignal(vel = Pose2d(x = x.roadrunner().raw, y = y.roadrunner().raw, heading = turn.roadrunner().raw)))
}

fun MecanumDrive.mecanumDrive(x: RRVelocity, y: RRVelocity, turn: AngularVelocity) = mecanumDrive(x.roadrunner(), y.roadrunner(), turn.roadrunner())
fun MecanumDrive.mecanumDrive(x: Velocity, y: Velocity, turn: AngularVelocity) = mecanumDrive(x.roadrunner(), y.roadrunner(), turn.roadrunner())