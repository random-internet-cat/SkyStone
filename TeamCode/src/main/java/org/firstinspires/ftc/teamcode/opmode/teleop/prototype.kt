package org.firstinspires.ftc.teamcode.opmode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.hardware.arm.PrototypeArm
import org.firstinspires.ftc.teamcode.hardware.drive.arcadeDrive
import org.firstinspires.ftc.teamcode.hardware.drive.tank.TankDrive
import org.firstinspires.ftc.teamcode.hardware.intake.PrototypeIntake
import org.firstinspires.ftc.teamcode.hardware.provider.makePrototypeHardware
import org.firstinspires.ftc.teamcode.util.units.DegreesPoint

@TeleOp
class PrototypeTeleop : LinearOpMode() {
    private fun updateDrive(gamepad: Gamepad, drive: TankDrive) {
        val linearPower = gamepad.left_stick_y.toDouble()
        val turnPower = gamepad.right_stick_x.toDouble()

        drive.arcadeDrive(linearPower = linearPower, turnPower = turnPower)
    }

    private fun updateIntake(gamepad: Gamepad, intake: PrototypeIntake) {
        when {
            gamepad.left_bumper -> intake.intake()
            gamepad.right_bumper -> intake.outtake()
            else -> intake.stop()
        }
    }

    private enum class ArmState {
        COLLECT, STAGE0, STAGE1
    }

    private fun updateArmRotator(state: ArmState, rotator: PrototypeArm.Rotator) {
        when(state) {
            ArmState.COLLECT -> rotator.moveToCollect()
            ArmState.STAGE0 -> rotator.moveToStage0()
            ArmState.STAGE1 -> rotator.moveToStage1()
        }
    }

    private fun updateArmWrist(state: ArmState, wrist: PrototypeArm.Wrist) {
        when(state) {
            ArmState.COLLECT -> wrist.moveToCollect()
            ArmState.STAGE0 -> wrist.moveToStage0()
            ArmState.STAGE1 -> wrist.moveToStage1()
        }
    }

    private enum class ClampState {
        OPEN, CLOSED
    }

    private fun ClampState.next(): ClampState {
        return when(this) {
            ClampState.OPEN -> ClampState.CLOSED
            ClampState.CLOSED -> ClampState.OPEN
        }
    }

    private var _clampState: ClampState = ClampState.OPEN
    private var _preventClampUpdate: Boolean = false

    private fun updateArmClamp(gamepad: Gamepad, clamp: PrototypeArm.Clamp) {
        if (gamepad.x) {
            if (!_preventClampUpdate) {
                _clampState = _clampState.next()
            }

            _preventClampUpdate = true
        } else {
            _preventClampUpdate = false
        }

        when(_clampState) {
            ClampState.OPEN -> clamp.open()
            ClampState.CLOSED -> clamp.close()
        }
    }

    private fun updateArm(gamepad: Gamepad, arm: PrototypeArm) {
        val newState = when {
            gamepad.a -> ArmState.COLLECT
            gamepad.b -> ArmState.STAGE0
            gamepad.y -> ArmState.STAGE1
            else -> null
        }

        if (newState != null) {
            updateArmRotator(newState, arm.rotator)
            updateArmWrist(newState, arm.wrist)
        }

        updateArmClamp(gamepad, arm.clamp)

        arm.update()
    }

    override fun runOpMode() {
        val hardware = makePrototypeHardware(hardwareMap)
        val drive = hardware.drive
        val intake = hardware.intake
        val arm = hardware.arm

        waitForStart()

        while (opModeIsActive()) {
            updateDrive(gamepad1, drive)
            updateIntake(gamepad1, intake)
            updateArm(gamepad2, arm)

            telemetry.addData("Arm angle (deg)", DegreesPoint(arm.rotator.currentAngle()).raw)
        }
    }
}