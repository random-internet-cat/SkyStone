package org.firstinspires.ftc.teamcode.opmode.teleop

import com.acmerobotics.dashboard.FtcDashboard
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
    private val dashboard = FtcDashboard.getInstance()

    private fun driveInputRamp(joystickInput: Double): Double {
        val cutoff = joystickInput.cutoffToZero()
        return (cutoff * cutoff).withSign(cutoff)
    }

    private fun driveInputRamp(joystickInput: Float) = driveInputRamp(joystickInput.toDouble())

    private fun handleDriveInputs(gamepad: Gamepad, drive: MecanumDrive, maxDriveRPM: RRAngularVelocity, maxVel: RRVelocity) {
        val x = driveInputRamp(gamepad.left_stick_y) * maxVel
        val y = driveInputRamp(gamepad.left_stick_x) * maxVel
        val turn = gamepad.right_stick_x.toDouble().cutoffToZero() * maxDriveRPM

        drive.mecanumDrive(x = x, y = y, turn = turn)
    }

    private fun handleDriveInputs(gamepad: Gamepad, drive: MecanumDrive, driveConfig: MecanumDriveConfig = drive.config)  = handleDriveInputs(gamepad, drive, driveConfig.maxDriveRPM(), driveConfig.maxVelocity())

    private fun handleFoundationMoverInputs(gamepad: Gamepad, foundationMover: BaseFoundationMover) {
        when {
            gamepad.dpad_down -> foundationMover.grab()
            gamepad.dpad_up -> foundationMover.release()
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
        when {
            gamepad.a -> {
                if (!_armMotionLastTick) {
                    val nextUpState = vertical.mostRecentAutomaticState().nextUp
                    if (nextUpState != null) vertical.moveToState(nextUpState)
                }

                _armMotionLastTick = true
            }

            gamepad.b -> {
                if (!_armMotionLastTick) {
                    val nextDownState = vertical.mostRecentAutomaticState().nextDown
                    if (nextDownState != null) vertical.moveToState(nextDownState)
                }

                _armMotionLastTick = true
            }

            else -> {
                vertical.stopIfManual()
                _armMotionLastTick = false
            }
        }
    }

    private fun handleArmHorizontalInputs(gamepad: Gamepad, horizontal: MarkIArm.HorizontalControl) {
        when {
            gamepad.x -> horizontal.moveOut()
            gamepad.y -> horizontal.moveIn()
            else -> horizontal.stop()
        }
    }

    private fun handleArmClampInputs(gamepad: Gamepad, clamp: MarkIArm.Clamp) {
        when {
            gamepad.dpad_left -> clamp.close()
            gamepad.dpad_right -> clamp.open()
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
            handleArmInputs(gamepad1, arm)

            telemetry.addData("Arm state", arm.vertical.currentAutomaticState() ?: "Manual")
            telemetry.update()
            hardware.update()
        }
    }
}