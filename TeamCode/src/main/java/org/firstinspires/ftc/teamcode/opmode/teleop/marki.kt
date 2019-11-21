package org.firstinspires.ftc.teamcode.opmode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.hardware.foundation_mover.BaseFoundationMover
import org.firstinspires.ftc.teamcode.hardware.arm.MarkIArm
import org.firstinspires.ftc.teamcode.hardware.drive.brakeOnZeroPower
import org.firstinspires.ftc.teamcode.hardware.drive.enableEncoders
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.*
import org.firstinspires.ftc.teamcode.hardware.intake.MarkIIntake
import org.firstinspires.ftc.teamcode.hardware.intake_release.MarkIIntakeRelease
import org.firstinspires.ftc.teamcode.hardware.provider.makeMarkIHardware
import org.firstinspires.ftc.teamcode.util.cutoffToZero
import org.firstinspires.ftc.teamcode.util.roadrunner.RRAngularVelocity
import org.firstinspires.ftc.teamcode.util.roadrunner.RRVelocity
import org.firstinspires.ftc.teamcode.util.roadrunner.roadrunner
import org.firstinspires.ftc.teamcode.util.units.times

@TeleOp
class MarkITeleop : LinearOpMode() {
    private val dashboard = FtcDashboard.getInstance()

    private fun handleDriveInputs(gamepad: Gamepad, drive: MecanumDrive, maxDriveRPM: RRAngularVelocity, maxVel: RRVelocity) {
        val x = gamepad.left_stick_y.toDouble().cutoffToZero() * maxVel
        val y = gamepad.left_stick_x.toDouble().cutoffToZero() * maxVel
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

    private fun handleArmRotatorInputs(gamepad: Gamepad, rotator: MarkIArm.Rotator) {
        when {
            gamepad.a -> rotator.moveToCollect()
            gamepad.b -> rotator.moveToStage0()
        }
    }

    private fun handleArmWristInputs(gamepad: Gamepad, wrist: MarkIArm.Wrist) {
        when {
            gamepad.x -> wrist.moveToCollect()
            gamepad.y -> wrist.moveToStage0()
        }
    }

    private fun handleArmClampInputs(gamepad: Gamepad, clamp: MarkIArm.Clamp) {
        when {
            gamepad.dpad_left -> clamp.close()
            gamepad.dpad_right -> clamp.open()
        }
    }

    private fun handleArmInputs(gamepad: Gamepad, arm: MarkIArm) {
        handleArmRotatorInputs(gamepad, arm.rotator)
        handleArmWristInputs(gamepad, arm.wrist)
        handleArmClampInputs(gamepad, arm.clamp)
    }

    private fun handleIntakeReleaseInputs(gamepad: Gamepad, release: MarkIIntakeRelease) {
        when {
            gamepad.right_bumper -> release.release()
        }
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

        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)

        waitForStart()

        while (opModeIsActive()) {
            handleDriveInputs(gamepad1, drive, maxDriveRPM = maxDriveRPM.roadrunner(), maxVel = maxVel.roadrunner())
            handleFoundationMoverInputs(gamepad1, foundationMover)
            handleIntakeInputs(gamepad1, intake)
            handleArmInputs(gamepad1, arm)
            handleIntakeReleaseInputs(gamepad1, hardware.intakeRelease)

            telemetry.update()
            hardware.update()
        }
    }
}