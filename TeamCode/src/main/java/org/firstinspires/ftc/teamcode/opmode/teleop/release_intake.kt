package org.firstinspires.ftc.teamcode.opmode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.hardware.provider.MarkIHardwareProvider

@TeleOp
class ReleaseIntake : LinearOpMode() {
    override fun runOpMode() {
        val intakeFlippers = MarkIHardwareProvider.makeIntakeFlippers(hardwareMap)

        waitForStart()

        intakeFlippers.release()

        while (opModeIsActive()) {}
    }
}