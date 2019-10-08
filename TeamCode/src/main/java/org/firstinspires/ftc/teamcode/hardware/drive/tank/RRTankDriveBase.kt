package org.firstinspires.ftc.teamcode.hardware.drive.tank

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.drive.TankDrive
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower
import com.acmerobotics.roadrunner.followers.TrajectoryFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.util.DcMotorFeedforward
import org.firstinspires.ftc.teamcode.util.roadrunner.*
import org.firstinspires.ftc.teamcode.util.units.*

/*
 * Base class with shared functionality for sample tank drives. All hardware-specific details are
 * handled in subclasses.
 */

data class TankDrivetrain(val trackWidth: Distance)
data class TankDrivePID(val axialPID: PIDCoefficients, val headingPID: PIDCoefficients, val crossTrackPID: PIDCoefficients)

@Config
abstract class RRTankDriveBase : TankDrive {
    private val dashboard: FtcDashboard
    private val clock: NanoClock

    private var mode: Mode? = null

    private val turnController: PIDFController
    private var turnProfile: MotionProfile? = null
    private var turnStart: Time = Time.zero()

    private val constraints: DriveConstraints
    private val follower: TrajectoryFollower

    constructor(drivetrain: TankDrivetrain, pid: TankDrivePID, feedforward: DcMotorFeedforward, baseConstraints: DriveConstraints) : super(feedforward.kV, feedforward.kA, feedforward.kStatic, drivetrain.trackWidth.roadrunner().raw) {
        this.constraints = TankConstraints(baseConstraints, drivetrain.trackWidth)
        this.follower = TankPIDVAFollower(pid.axialPID, pid.crossTrackPID)
        this.turnController = PIDFController(pid.headingPID)

        turnController.setInputBounds(0.0, 2 * Math.PI)
    }

    fun lastError(): Pose2d {
        when (mode) {
            Mode.FOLLOW_TRAJECTORY -> return follower.lastError
            Mode.TURN -> return Pose2d(0.0, 0.0, turnController.lastError)
            Mode.IDLE -> return Pose2d()
        }
        throw AssertionError()
    }

    fun isBusy(): Boolean = mode != Mode.IDLE

    private fun currentTime() = Seconds(clock.seconds())

    enum class Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    init {
        dashboard = FtcDashboard.getInstance()
        clock = NanoClock.system()

        mode = Mode.IDLE
    }

    fun trajectoryBuilder(): TrajectoryBuilder {
        return TrajectoryBuilder(poseEstimate, constraints)
    }

    fun turn(angle: Angle) {
        val angle = Radians(angle)
        val heading = RadiansPoint(poseEstimate.heading)
        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
            MotionState(heading.raw, 0.0, 0.0, 0.0),
            MotionState((heading + angle).raw, 0.0, 0.0, 0.0),
            constraints.maxAngVel,
            constraints.maxAngAccel,
            constraints.maxAngJerk
        )
        turnStart = currentTime()
        mode = Mode.TURN
    }

    fun turnSync(angle: Angle) {
        turn(angle)
        waitForIdle()
    }

    fun followTrajectory(trajectory: Trajectory) {
        follower.followTrajectory(trajectory)
        mode = Mode.FOLLOW_TRAJECTORY
    }

    fun followTrajectorySync(trajectory: Trajectory) {
        followTrajectory(trajectory)
        waitForIdle()
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

        when (mode) {
            Mode.IDLE -> {
            }
            Mode.TURN -> {
                val t = currentTime() - turnStart

                val targetState = turnProfile!![t]
                val targetOmega = targetState.v
                val targetAlpha = targetState.a
                val correction = turnController.update(currentPose.heading, targetOmega)

                setDriveSignal(DriveSignal(Pose2d(
                    0.0, 0.0, targetOmega + correction
                ), Pose2d(
                    0.0, 0.0, targetAlpha
                )))

                if (t >= RRTime(turnProfile!!.duration())) {
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

                fieldOverlay.setStroke("#3F51B5")
                fieldOverlay.fillCircle(currentPose.x, currentPose.y, 3.0)

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE
                    setDriveSignal(DriveSignal())
                }
            }
        }// do nothing

        dashboard.sendTelemetryPacket(packet)
    }

    fun waitForIdle() {
        while (!Thread.currentThread().isInterrupted && isBusy()) {
            update()
        }
    }

    abstract fun getPIDCoefficients(runMode: DcMotor.RunMode): PIDCoefficients

    abstract fun setPIDCoefficients(runMode: DcMotor.RunMode, coefficients: PIDCoefficients)
}