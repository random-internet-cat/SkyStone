package org.firstinspires.ftc.teamcode.opmode.testing.drive

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.hardware.provider.MarkIHardwareProvider
import org.firstinspires.ftc.teamcode.util.roadrunner.RRDistance
import org.firstinspires.ftc.teamcode.util.units.Inches
import org.firstinspires.ftc.teamcode.util.units.Meters

@Autonomous
class DrivePosReader : LinearOpMode() {
    override fun runOpMode() {
        val drive = MarkIHardwareProvider.makeDrive(hardwareMap).roadrunner()

        waitForStart()

        while (opModeIsActive()) {
            val wheelPos = drive.getWheelPositions().map { RRDistance(it) }.map { "${Inches(it).raw} inches" }

            telemetry.addData("Front left", wheelPos[0])
            telemetry.addData("Back left", wheelPos[1])
            telemetry.addData("Back right", wheelPos[2])
            telemetry.addData("Front right", wheelPos[3])

            telemetry.update()
            drive.update()
        }
    }
}