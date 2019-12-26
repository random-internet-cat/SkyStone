package org.firstinspires.ftc.teamcode.drive.mecanum

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.followers.TrajectoryFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.util.DcMotorCharacterization
import org.firstinspires.ftc.teamcode.util.roadrunner.*

import org.firstinspires.ftc.teamcode.util.units.*

/*
 * Base class with shared functionality for sample mecanum drives. All hardware-specific details are
 * handled in subclasses.
 */

data class MecanumDrivetrainConfig(val trackWidth: Distance, val wheelBase: Distance)
data class MecanumPID(val translationalPID: PIDCoefficients, val headingPID: PIDCoefficients)

@Config
abstract class RRMecanumDriveBase(drivetrainConfig: MecanumDrivetrainConfig, pid: MecanumPID, characterization: DcMotorCharacterization, baseConstraints: DriveConstraints) : MecanumDrive(characterization.kV, characterization.kA, characterization.kStatic, drivetrainConfig.trackWidth.roadrunner().raw, drivetrainConfig.wheelBase.roadrunner().raw) {
    private val dashboard: FtcDashboard = FtcDashboard.getInstance()
    private val clock = DefaultSystemClock

    private var mode: Mode = Mode.IDLE

    private val turnController: PIDFController
    private var turnProfile: MotionProfile? = null
    private lateinit var turnStart: SystemTime

    private val constraints: DriveConstraints
    private val follower: TrajectoryFollower

    private var lastWheelPositions: List<Double>? = null
    private lateinit var lastTimestamp: SystemTime

    private val _mutableTranslationalPID: RRPIDCoefficients
    private val _mutableHeadingPID: RRPIDCoefficients

    init {
        _mutableTranslationalPID = RRPIDCoefficients(pid.translationalPID)
        _mutableHeadingPID = RRPIDCoefficients(pid.headingPID)

        dashboard.setTelemetryTransmissionInterval(25)

        constraints = MecanumConstraints(baseConstraints, drivetrainConfig.trackWidth, drivetrainConfig.wheelBase)
        follower = HolonomicPIDVAFollower(_mutableTranslationalPID, _mutableTranslationalPID, _mutableHeadingPID)
        turnController = PIDFController(_mutableHeadingPID)

        turnController.setInputBounds(0.0, 2 * Math.PI)
    }

    fun translationalPID() = PIDCoefficients(_mutableTranslationalPID)

    fun setTranslationalPID(newPID: PIDCoefficients) {
        _mutableTranslationalPID.assign(newPID)
    }

    fun headingPID() = PIDCoefficients(_mutableHeadingPID)

    fun setHeadingPID(newPID: PIDCoefficients) {
        _mutableHeadingPID.assign(newPID)
    }

    fun pid() = MecanumPID(translationalPID = translationalPID(), headingPID = headingPID())

    fun setPID(newPID: MecanumPID) {
        setHeadingPID(newPID.headingPID)
        setTranslationalPID(newPID.translationalPID)
    }

    fun lastError(): Pose2d = when (mode) {
        Mode.FOLLOW_TRAJECTORY -> follower.lastError
        Mode.TURN -> Pose2d(0.0, 0.0, turnController.lastError)
        Mode.IDLE -> Pose2d()
    }

    fun isBusy(): Boolean = mode != Mode.IDLE

    private fun currentTime() = clock.currentTime()

    enum class Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    fun trajectoryBuilder(): TrajectoryBuilder {
        return TrajectoryBuilder(poseEstimate, constraints)
    }

    fun turn(angle: Angle) {
        val heading = Radians(poseEstimate.heading)

        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
            MotionState(Radians(heading).raw, 0.0, 0.0, 0.0),
            MotionState(Radians(heading + angle).raw, 0.0, 0.0, 0.0),
            constraints.maxAngVel,
            constraints.maxAngAccel,
            constraints.maxAngJerk
        )
        turnStart = currentTime()
        mode = Mode.TURN
    }

    private fun brake() {
        setMotorPowers(0.0, 0.0, 0.0, 0.0)
    }

    fun turnSync(angle: Angle) {
        turn(angle)
        waitForIdle()
        brake()
    }

    fun followTrajectory(trajectory: Trajectory) {
        follower.followTrajectory(trajectory)
        mode = Mode.FOLLOW_TRAJECTORY
    }

    fun followTrajectorySync(trajectory: Trajectory) {
        followTrajectory(trajectory)
        waitForIdle()
        brake()
    }

    fun update() {
        updatePoseEstimate()

        val currentPose = poseEstimate
        val lastError = lastError()

        val packet = TelemetryPacket()
        val fieldOverlay = packet.fieldOverlay()

        packet.put("mode", mode)

        packet.put("x", currentPose.x)
        packet.put("y", currentPose.y)
        packet.put("heading", currentPose.heading)

        packet.put("xError", lastError.x)
        packet.put("yError", lastError.y)
        packet.put("headingError", lastError.heading)

        run {
            fieldOverlay.setStroke("#3F51B5")
            val poseX = Inches(RRDistance(currentPose.x)).raw
            val poseY = Inches(RRDistance(currentPose.y)).raw
            fieldOverlay.fillCircle(poseX, poseY, 3.0)

            val (hX, hY) = poseEstimate.headingVec()
            fieldOverlay.strokeLine(poseX, poseY, poseX + hX * 4.5, poseY + hY * 4.5)
        }

        when (mode) {
            Mode.IDLE -> {
                // do nothing
            }

            Mode.TURN -> {
                val t = currentTime() - turnStart

                val targetState = turnProfile!![t]

                turnController.targetPosition = targetState.x

                val targetOmega = targetState.v
                val targetAlpha = targetState.a
                val correction = turnController.update(currentPose.heading, targetOmega)

                setDriveSignal(DriveSignal(Pose2d(
                    0.0, 0.0, targetOmega + correction
                ), Pose2d(
                    0.0, 0.0, targetAlpha
                )))

                if (t >= RRDuration(turnProfile!!.duration())) {
                    mode = Mode.IDLE
                    setDriveSignal(DriveSignal())
                }
            }

            Mode.FOLLOW_TRAJECTORY -> {
                setDriveSignal(follower.update(currentPose))

                val trajectory = follower.trajectory

                fieldOverlay.setStrokeWidth(1)
                fieldOverlay.setStroke("4CAF50")
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.path)

                fieldOverlay.setStroke("#F44336")
                val t = follower.elapsedTime()
                DashboardUtil.drawRobot(fieldOverlay, trajectory[t])

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE
                    setDriveSignal(DriveSignal())
                }
            }
        }

        dashboard.sendTelemetryPacket(packet)
    }

    fun waitForIdle() {
        while (!Thread.currentThread().isInterrupted && isBusy()) {
            update()
        }
    }

    open fun getWheelVelocities(): List<Double> {
        val lastWheelPositions = lastWheelPositions
        val positions = getWheelPositions()
        val currentTimestamp = currentTime()

        val velocities = mutableListOf<Double>()
        if (lastWheelPositions != null) {
            val dt = currentTimestamp - lastTimestamp
            for (i in positions.indices) {
                velocities.add((RRDistance(positions[i] - lastWheelPositions[i]) / dt).roadrunner().raw)
            }
        } else {
            for (i in positions.indices) {
                velocities.add(Velocity.zero().roadrunner().raw)
            }
        }

        this.lastTimestamp = currentTimestamp
        this.lastWheelPositions = positions

        return velocities
    }


    abstract fun getPIDCoefficients(runMode: DcMotor.RunMode): PIDCoefficients

    abstract fun setPIDCoefficients(runMode: DcMotor.RunMode, coefficients: PIDCoefficients)
}

fun RRMecanumDriveBase.setCurrentPosition(position: PositionVector) {
    localizer.poseEstimate = Pose2d(position.roadrunner(), externalHeading)
    this.updatePoseEstimate()
}