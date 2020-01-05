package org.firstinspires.ftc.teamcode.opmode.testing

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.hardware.provider.MarkIHardwareProvider
import org.firstinspires.ftc.teamcode.util.units.Degrees
import org.firstinspires.ftc.teamcode.util.units.Radians

@Autonomous
class HeadingTest : LinearOpMode() {
    override fun runOpMode() {
        val drive = MarkIHardwareProvider.makeDrive(hardwareMap).roadrunner()

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addData("Heading (deg)", Degrees(Radians(drive.poseEstimate.heading)).raw)
            telemetry.update()
        }
    }
}