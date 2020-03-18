package org.firstinspires.ftc.teamcode.opmode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.hardware.MarkIHardware
import org.firstinspires.ftc.teamcode.hardware.foundation_mover.BaseFoundationMover
import org.firstinspires.ftc.teamcode.hardware.arm.MarkIArm
import org.firstinspires.ftc.teamcode.hardware.capstone_dropper.MarkICapstoneDropper
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.*
import org.firstinspires.ftc.teamcode.hardware.drive.stop
import org.firstinspires.ftc.teamcode.hardware.intake.MarkIIntake
import org.firstinspires.ftc.teamcode.hardware.provider.makeMarkIHardware
import org.firstinspires.ftc.teamcode.util.cutoffToZero
import org.firstinspires.ftc.teamcode.util.roadrunner.RRAngularVelocity
import org.firstinspires.ftc.teamcode.util.roadrunner.RRVelocity
import org.firstinspires.ftc.teamcode.util.roadrunner.roadrunner
import org.firstinspires.ftc.teamcode.util.units.times
import kotlin.math.abs
import kotlin.math.withSign

@TeleOp
class MarkITeleop : LinearOpMode() {
    private fun driveInputRamp(joystickInput: Double): Double {
        val cutoff = joystickInput.cutoffToZero()
        return (cutoff * cutoff).withSign(cutoff)
    }

    private fun driveInputRamp(joystickInput: Float) = driveInputRamp(joystickInput.toDouble())

    private var _isSlow: Boolean = false
    private var _slowFromIntaking : Boolean = false
    private var _slowPressedLastTick: Boolean = false

    private fun shouldDriveSlow(gamepad: Gamepad): Boolean {
        val slowButtonIsPressed = gamepad.a

        if (slowButtonIsPressed && !_slowPressedLastTick) {
            _isSlow = !_isSlow
        }

        _slowPressedLastTick = slowButtonIsPressed

        return _isSlow || _slowFromIntaking
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
            gamepad.left_trigger > 0.1 -> {
                _slowFromIntaking = true
                intake.intake()
            }
            gamepad.right_trigger > 0.1 -> intake.outtake()
            else -> {
                _slowFromIntaking = false
                intake.stop()
            }
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

            abs(manualDirection) > 0.05 -> {
                vertical.manuallyMoveWithPower(manualDirection.toDouble())
            }

            else -> {
                vertical.stopIfManual()
                _armMotionLastTick = false
            }
        }
    }

    private fun handleArmHorizontalAndClampInputs(gamepad: Gamepad, horizontal: MarkIArm.HorizontalControl, clamp: MarkIArm.Clamp) {
        // right stick for (small) adjustments
        val manualPower = gamepad.right_stick_y.toDouble()
        val useManual = abs(manualPower) > 0.1

        if (useManual) {
            horizontal.power(-manualPower * .8)
        } else {
            // triggers for max and min presets
            when {
                gamepad.right_trigger > 0.1 -> horizontal.moveAllTheWayOut()

                gamepad.left_trigger > 0.1 -> {
                    horizontal.moveAllTheWayIn()
                    if (!horizontal.closeToIn()) {
                        clamp.open()
                    }
                }

                else -> horizontal.stopIfManual()
            }
        }
    }

    private fun handleFullOuttakeInputs(gamepad: Gamepad, hardware: MarkIHardware) {
        val arm = hardware.arm
        val vertical = arm.vertical
        val horizontal = arm.horizontal
        val clamp = arm.clamp
        val drive = hardware.drive

        when {

            // Retract horizontal & vertical
            gamepad.y -> {
                if (!_armMotionLastTick) {
                    vertical.moveToState(MarkIArm.VerticalControl.State.CollectState)
                    horizontal.moveAllTheWayIn()
                    clamp.open()
                }
            }

            // Un-clamp, clear stack and full retract if horizontal is somewhat out
            gamepad.a -> {
                if (!_armMotionLastTick
                    && horizontal.motor.currentPosition >= MarkIArm.HorizontalControl.MAX_ENCODER_VALUE * 0.75
                    && vertical.canGoOneBlockUp()) {

                    drive.stop()

                    // Open clamp if closed
                    if (clamp.servoPosition() == MarkIArm.Clamp.CLOSED_POSITION) {
                        clamp.open()
                        sleep(300)
                    }

                    // Clear stack with horizontal and rise to next stage
                    horizontal.moveToClearStack()
                    vertical.moveToOneBlockUp()
                    sleep(450)

                    // Retract horizontal all the way in
                    horizontal.moveAllTheWayIn()
                    sleep(400)

                    // Retract vertical all the way in, and return control to player while doing so
                    vertical.moveToState(MarkIArm.VerticalControl.State.CollectState)

                }
            }

        }
    }

    private fun handleArmClampSpecificInputs(gamepad: Gamepad, clamp: MarkIArm.Clamp) {
        when {
            gamepad.x -> clamp.close()
            gamepad.b -> clamp.open()
        }
    }

    private fun handleCapstoneDropperInputs(gamepad: Gamepad, dropper: MarkICapstoneDropper) {
        when {
            gamepad.dpad_right -> dropper.moveInWay()
            gamepad.dpad_left -> dropper.moveOutOfWay()
        }
    }

    private fun handleArmInputs(gamepad: Gamepad, hardware: MarkIHardware) {
        val arm = hardware.arm

        handleArmHorizontalAndClampInputs(gamepad, arm.horizontal, arm.clamp)
        handleArmVerticalInputs(gamepad, arm.vertical)
        handleFullOuttakeInputs(gamepad, hardware)
        handleArmClampSpecificInputs(gamepad, arm.clamp)
    }

    override fun runOpMode() {
        val hardware = makeMarkIHardware(hardwareMap)
        val arm = hardware.arm
        val drive = hardware.drive
        val foundationMover = hardware.foundationMover
        val drivetrain = drive.drivetrain
        val driveConfig = drive.config
        val maxDriveRPM = driveConfig.maxDriveRPM().roadrunner()
        val maxVel = driveConfig.maxVelocity().roadrunner() * 1.3
        val intake = hardware.intake
        val capstoneDropper = hardware.capstoneDropper

        waitForStart()

        arm.vertical.moveToState(MarkIArm.VerticalControl.State.CollectState)
        arm.clamp.open()

        while (opModeIsActive()) {
            handleDriveInputs(gamepad1, drive, maxDriveRPM = maxDriveRPM.roadrunner(), maxVel = maxVel.roadrunner())
            handleFoundationMoverInputs(gamepad1, foundationMover)
            handleIntakeInputs(gamepad1, intake)
            handleArmInputs(gamepad2, hardware)
            handleCapstoneDropperInputs(gamepad2, capstoneDropper)

            telemetry.addData("Arm state", arm.vertical.currentAutomaticState() ?: "Manual")
            telemetry.addData("Is slow", _isSlow)
            telemetry.addData("Horizontal ticks", arm.horizontal.motor.currentPosition)
            telemetry.addData("Vertical ticks", arm.vertical.encoderPosition())
            telemetry.update()
            hardware.update()
        }
    }
}