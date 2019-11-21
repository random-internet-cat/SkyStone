package org.firstinspires.ftc.teamcode.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.hardware.provider.makeMarkIIntakeRelease
import org.firstinspires.ftc.teamcode.util.getServo

@Autonomous(name = "Release Intake")
class ReleaseIntake : LinearOpMode() {
    override fun runOpMode() {
        val release = makeMarkIIntakeRelease(hardwareMap)
        waitForStart()

        release.release()

        while (opModeIsActive()) {}
    }
}