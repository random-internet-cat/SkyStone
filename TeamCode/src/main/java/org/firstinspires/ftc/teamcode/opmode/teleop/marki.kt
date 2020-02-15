package org.firstinspires.ftc.teamcode.opmode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.hardware.foundation_mover.BaseFoundationMover
import org.firstinspires.ftc.teamcode.hardware.arm.MarkIArm
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.*
import org.firstinspires.ftc.teamcode.hardware.imu.InternalIMU
import org.firstinspires.ftc.teamcode.hardware.intake.MarkIIntake
import org.firstinspires.ftc.teamcode.hardware.provider.makeMarkIHardware
import org.firstinspires.ftc.teamcode.util.cutoffToZero
import org.firstinspires.ftc.teamcode.util.getIMU
import org.firstinspires.ftc.teamcode.util.roadrunner.RRAngularVelocity
import org.firstinspires.ftc.teamcode.util.roadrunner.RRVelocity
import org.firstinspires.ftc.teamcode.util.roadrunner.roadrunner
import org.firstinspires.ftc.teamcode.util.units.*
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.withSign

@TeleOp
class MarkITeleop : LinearOpMode() {
    private fun driveInputRamp(joystickInput: Double): Double {
        val cutoff = joystickInput.cutoffToZero()
        return (cutoff * cutoff).withSign(cutoff)
    }

    private fun driveInputRamp(joystickInput: Float) = driveInputRamp(joystickInput.toDouble())

    private var _isSlow: Boolean = false
    private var _slowPressedLastTick: Boolean = false

    private fun shouldDriveSlow(gamepad: Gamepad): Boolean {
        val slowButtonIsPressed = gamepad.a

        if (slowButtonIsPressed && !_slowPressedLastTick) {
            _isSlow = !_isSlow
        }

        _slowPressedLastTick = slowButtonIsPressed

        return _isSlow
    }

    private fun handleDriveFieldOrientedInputs(gamepad: Gamepad, drive: MecanumDrive, imu: InternalIMU, maxDriveRPM: RRAngularVelocity, maxVel: RRVelocity) {
        val isSlowMode = shouldDriveSlow(gamepad)
        val slowFactor = if (isSlowMode) 0.3 else 1.0

        // Field orienting the drive
        val inputFieldAngle = RadiansPoint(atan2(gamepad.left_stick_x * -1, gamepad.left_stick_y * -1))

        val robotRelativeAngle = inputFieldAngle - imu.heading()

        val velocityMagnitude = driveInputRamp(hypot(gamepad.left_stick_x, gamepad.left_stick_y).coerceAtMost(1.0f))

        // Applying input ramp, max velocity, and slowness to the input
        val x = cos(robotRelativeAngle) * velocityMagnitude * maxVel * slowFactor
        val y = sin(robotRelativeAngle) * velocityMagnitude * maxVel * slowFactor

        // Apply max velocity, cutoff, and slowness to turning
        val turn = (gamepad.right_stick_x * -1).toDouble().cutoffToZero() * maxDriveRPM * slowFactor

        drive.mecanumDrive(x = x, y = y, turn = turn)
    }

    private fun handleDriveInputs(gamepad: Gamepad, drive: MecanumDrive, maxDriveRPM: RRAngularVelocity, maxVel: RRVelocity) {
        val isSlowMode = shouldDriveSlow(gamepad)
        val slowFactor = if (isSlowMode) 0.3 else 1.0

        val x = driveInputRamp(gamepad.left_stick_y * -1) * maxVel * slowFactor
        val y = driveInputRamp(gamepad.left_stick_x * -1) * maxVel * slowFactor
        val turn = (gamepad.right_stick_x * -1).toDouble().cutoffToZero() * maxDriveRPM * slowFactor

        drive.mecanumDrive(x = x, y = y, turn = turn)
    }

    private fun handleDriveInputs(gamepad: Gamepad, drive: MecanumDrive, driveConfig: MecanumDriveConfig = drive.config)  = handleDriveInputs(gamepad, drive, driveConfig.maxDriveRPM(), driveConfig.maxVelocity())

    private fun handleFoundationMoverInputs(gamepad: Gamepad, foundationMover: BaseFoundationMover) {
        when {
            gamepad.dpad_down -> foundationMover.grabBoth()
            gamepad.dpad_up -> foundationMover.moveBothToOutOfTheWay()
        }
    }

    private fun handleIntakeInputs(gamepad: Gamepad, intake: MarkIIntake) {
        when {
            gamepad.left_trigger > 0.1 -> intake.intake()
            gamepad.right_trigger > 0.1 -> intake.outtake()
            else -> intake.stop()
        }
    }

    private var _armMotionLastTick = false

    private fun handleArmVerticalInputs(gamepad: Gamepad, vertical: MarkIArm.VerticalControl) {
        val manualDirection = gamepad.left_stick_y

        when {
            gamepad.right_bumper -> {
                if (!_armMotionLastTick) {
                    val nextUpState = vertical.mostRecentAutomaticState().nextUp
                    if (nextUpState != null) vertical.moveToState(nextUpState)
                }

                _armMotionLastTick = true
            }

            gamepad.left_bumper -> {
                if (!_armMotionLastTick) {
                    val nextDownState = vertical.mostRecentAutomaticState().nextDown
                    if (nextDownState != null) vertical.moveToState(nextDownState)
                }

                _armMotionLastTick = true
            }

            manualDirection > 0.1 -> {
                vertical.manuallyMoveUp()
            }

            manualDirection < -0.1 -> {
                vertical.manuallyMoveDown()
            }

            else -> {
                vertical.stopIfManual()
                _armMotionLastTick = false
            }
        }
    }

    private fun handleArmHorizontalInputs(gamepad: Gamepad, horizontal: MarkIArm.HorizontalControl) {
        // right stick for (small) adjustments
        val manualPower = gamepad.right_stick_y.toDouble()
        val useManual = abs(manualPower) > 0.1

        if (useManual) {
            horizontal.power(manualPower)
        } else {
            // triggers for max and min presets
            when {
                gamepad.left_trigger > 0.1 -> horizontal.moveAllTheWayOut()
                gamepad.right_trigger > 0.1 -> horizontal.moveAllTheWayIn()

                else -> horizontal.stopIfManual()
            }
        }
    }

    private fun handleFullOuttakeInputs(gamepad: Gamepad, vertical: MarkIArm.VerticalControl, horizontal: MarkIArm.HorizontalControl, clamp: MarkIArm.Clamp) {
        when {

            // Retract horizontal & vertical
            gamepad.b -> {
                if (!_armMotionLastTick) {
                    vertical.moveToState(MarkIArm.VerticalControl.State.CollectState)
                    horizontal.moveAllTheWayIn()
                    clamp.open()
                }
            }

        }
    }

    private fun handleArmClampInputs(gamepad: Gamepad, clamp: MarkIArm.Clamp) {
        when {
            gamepad.a -> clamp.close()
            gamepad.x -> clamp.open()
        }
    }

    private fun handleArmInputs(gamepad: Gamepad, arm: MarkIArm) {
        handleArmHorizontalInputs(gamepad, arm.horizontal)
        handleArmVerticalInputs(gamepad, arm.vertical)
        handleFullOuttakeInputs(gamepad, arm.vertical, arm.horizontal, arm.clamp)
        handleArmClampInputs(gamepad, arm.clamp)
    }

    override fun runOpMode() {
        val hardware = makeMarkIHardware(hardwareMap)
        val arm = hardware.arm
        val drive = hardware.drive
        val foundationMover = hardware.foundationMover
        val drivetrain = drive.drivetrain
        val driveConfig = drive.config
        val maxDriveRPM = driveConfig.maxDriveRPM().roadrunner()
        val maxVel = driveConfig.maxVelocity().roadrunner()
        val intake = hardware.intake
        val imu = hardwareMap.getIMU()

        waitForStart()

        arm.vertical.moveToState(MarkIArm.VerticalControl.State.CollectState)
        arm.clamp.open()

        while (opModeIsActive()) {
            imu.update()

            //handleDriveInputs(gamepad1, drive, maxDriveRPM = maxDriveRPM.roadrunner(), maxVel = maxVel.roadrunner())
            handleDriveFieldOrientedInputs(gamepad1, drive, imu, maxDriveRPM = maxDriveRPM.roadrunner(), maxVel = maxVel.roadrunner())
            handleFoundationMoverInputs(gamepad1, foundationMover)
            handleIntakeInputs(gamepad1, intake)
            handleArmInputs(gamepad2, arm = arm)

            telemetry.addData("Arm state", arm.vertical.currentAutomaticState() ?: "Manual")
            telemetry.addData("Is slow", _isSlow)
            telemetry.addData("imu heading", imu.heading())
            telemetry.update()
            hardware.update()
        }
    }
}