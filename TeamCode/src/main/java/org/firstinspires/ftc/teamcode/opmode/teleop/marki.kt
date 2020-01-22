package org.firstinspires.ftc.teamcode.opmode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.hardware.foundation_mover.BaseFoundationMover
import org.firstinspires.ftc.teamcode.hardware.arm.MarkIArm
import org.firstinspires.ftc.teamcode.hardware.drive.brakeOnZeroPower
import org.firstinspires.ftc.teamcode.hardware.drive.enableEncoders
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.*
import org.firstinspires.ftc.teamcode.hardware.intake.MarkIIntake
import org.firstinspires.ftc.teamcode.hardware.provider.makeMarkIHardware
import org.firstinspires.ftc.teamcode.util.cutoffToZero
import org.firstinspires.ftc.teamcode.util.roadrunner.RRAngularVelocity
import org.firstinspires.ftc.teamcode.util.roadrunner.RRVelocity
import org.firstinspires.ftc.teamcode.util.roadrunner.roadrunner
import org.firstinspires.ftc.teamcode.util.units.times
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

    private fun handleDriveInputs(gamepad: Gamepad, drive: MecanumDrive, maxDriveRPM: RRAngularVelocity, maxVel: RRVelocity) {
        val isSlowMode = shouldDriveSlow(gamepad)
        val slowFactor = if (isSlowMode) 0.5 else 1.0

        val x = driveInputRamp(gamepad.left_stick_y * -1) * maxVel * slowFactor
        val y = driveInputRamp(gamepad.left_stick_x * -1) * maxVel * slowFactor
        val turn = (gamepad.right_stick_x * -1).toDouble().cutoffToZero() * maxDriveRPM * slowFactor

        drive.mecanumDrive(x = x, y = y, turn = turn)
    }

    private fun handleDriveInputs(gamepad: Gamepad, drive: MecanumDrive, driveConfig: MecanumDriveConfig = drive.config)  = handleDriveInputs(gamepad, drive, driveConfig.maxDriveRPM(), driveConfig.maxVelocity())

    private fun handleFoundationMoverInputs(gamepad: Gamepad, foundationMover: BaseFoundationMover) {
        when {
            gamepad.dpad_down -> foundationMover.grabBoth()
            gamepad.dpad_up -> foundationMover.releaseBoth()
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
        when {
            gamepad.left_trigger > 0.1 -> horizontal.moveOut()
            gamepad.right_trigger > 0.1 -> horizontal.moveIn()
            else -> horizontal.stop()
        }
    }

    private fun handleArmClampInputs(gamepad: Gamepad, clamp: MarkIArm.Clamp) {
        when {
            gamepad.left_bumper -> clamp.close()
            gamepad.right_bumper -> clamp.open()
        }
    }

    private fun handleArmInputs(gamepad: Gamepad, arm: MarkIArm) {
        handleArmHorizontalInputs(gamepad, arm.horizontal)
        handleArmVerticalInputs(gamepad, arm.vertical)
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

        drive.enableEncoders()
        drive.brakeOnZeroPower()

        waitForStart()

        while (opModeIsActive()) {
            handleDriveInputs(gamepad1, drive, maxDriveRPM = maxDriveRPM.roadrunner(), maxVel = maxVel.roadrunner())
            handleFoundationMoverInputs(gamepad1, foundationMover)
            handleIntakeInputs(gamepad1, intake)
            handleArmInputs(gamepad2, arm = arm)

            telemetry.addData("Arm state", arm.vertical.currentAutomaticState() ?: "Manual")
            telemetry.addData("Is slow", _isSlow)
            telemetry.update()
            hardware.update()
        }
    }
}