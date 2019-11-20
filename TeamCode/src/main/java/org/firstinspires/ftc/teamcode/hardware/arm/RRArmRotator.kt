package org.firstinspires.ftc.teamcode.hardware.arm

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.util.*
import org.firstinspires.ftc.teamcode.util.roadrunner.*
import org.firstinspires.ftc.teamcode.util.roadrunner.PIDFController
import org.firstinspires.ftc.teamcode.util.units.*

data class ArmMovementConstraints(val maxAngle: RRAnglePoint, val maxVel: RRAngularVelocity, val maxAccel: RRAngularAcceleration, val maxJerk: RRAngularJerk) {
    constructor(maxAngle: AnglePoint, maxVel: AngularVelocity, maxAccel: AngularAcceleration, maxJerk: AngularJerk) : this(maxAngle.roadrunner(), maxVel.roadrunner(), maxAccel.roadrunner(), maxJerk.roadrunner())

    // No max jerk overloads
    constructor(maxAngle: RRAnglePoint, maxVel: RRAngularVelocity, maxAccel: RRAngularAcceleration) : this(maxAngle, maxVel, maxAccel, AngularJerk.zero())
    constructor(maxAngle: AnglePoint, maxVel: AngularVelocity, maxAccel: AngularAcceleration) : this(maxAngle.roadrunner(), maxVel.roadrunner(), maxAccel.roadrunner())
}

/*
 * Hardware class for a rotary arm (for linearly-actuated mechanisms, see Elevator).
 */
class RRArmRotator(private val typedMotor: TypedMotor, pid: PIDCoefficients, val characterization: DcMotorCharacterization, var angleFeedforwardConstant: Double, private val movementConstraints: ArmMovementConstraints) {
    private lateinit var controller: PIDFController
    private val clock = NanoClock.system()
    private val angleOffset: AnglePoint

    private var desiredAngle: RadiansPoint = AnglePoint.zero()
    private var profile: MotionProfile? = null
    private var profileStartTime: Time = Time.zero()
    private val _pid: RRPIDCoefficients

    private fun currentTime() = Seconds(clock.seconds())

    fun isBusy(): Boolean {
        val profile = profile
        return profile != null && currentTime() - profileStartTime <= RRTime(profile.duration())
    }

    fun targetAngle() = desiredAngle
    fun currentAngle() = (typedMotor.anglePosition() - angleOffset) + AnglePoint.zero() // This is known to work, since angle at angleOffset is supposed to be 0

    fun setPID(newPID: PIDCoefficients) {
        _pid.kP = newPID.kP
        _pid.kI = newPID.kI
        _pid.kD = newPID.kD
    }

    fun pid() = PIDCoefficients(_pid)

    init {
        typedMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        typedMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        // note: if the arm is affected by a non-negligible constant force along the direction
        // of motion (e.g., gravity, kinetic friction, or a combination thereof), it may be
        // beneficial to compensate for it with gravity feedforward
        angleOffset = typedMotor.anglePosition().toRadians()

        _pid = RRPIDCoefficients(pid)
        controller = PIDFController(_pid, characterization, { raw -> angleFeedforwardConstant * cos(RRAnglePoint(raw)) }, clock)
    }

    fun moveToAngle(angle: AnglePoint) {
        val adjustedTarget = min(max(AnglePoint.zero(), angle), movementConstraints.maxAngle)

        val time = currentTime() - profileStartTime

        val start = if (isBusy()) profile!![time] else MotionState(desiredAngle, AngularVelocity.zero(), AngularAcceleration.zero(), AngularJerk.zero())
        val goal = MotionState(adjustedTarget, AngularVelocity.zero(), AngularAcceleration.zero(), AngularJerk.zero())

        profile = MotionProfileGenerator.generateSimpleMotionProfile(
            start, goal, movementConstraints.maxVel, movementConstraints.maxAccel, movementConstraints.maxJerk
        )

        profileStartTime = currentTime()

        this.desiredAngle = adjustedTarget
    }

    fun update() {
        val power: Double
        val currentAngle = currentAngle()

        if (isBusy()) {
            val profile = profile!!

            // following a profile
            val time = currentTime() - profileStartTime
            val state = profile[time]

            controller.targetPosition = state.x
            power = controller.update(currentAngle.roadrunner().raw, state.v, state.a)
        } else {
            // just hold the position
            controller.targetPosition = desiredAngle.roadrunner().raw
            power = controller.update(currentAngle.roadrunner().raw)
        }

        power(power)
    }

    fun power(power: Double) {
        typedMotor.setPower(power)
    }
}

fun RRArmRotator.power(power: Int) = power(power.toDouble())