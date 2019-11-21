package org.firstinspires.ftc.teamcode.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.hardware.provider.makeMarkIDrive
import org.firstinspires.ftc.teamcode.hardware.provider.makeMarkIHardware
import org.firstinspires.ftc.teamcode.util.roadrunner.RobotPosition
import org.firstinspires.ftc.teamcode.util.roadrunner.forward
import org.firstinspires.ftc.teamcode.util.roadrunner.splineTo
import org.firstinspires.ftc.teamcode.util.units.*

@Autonomous
class StraightTest : LinearOpMode() {
    override fun runOpMode() {
        val baseDrive = makeMarkIDrive(hardwareMap)

        val drive = baseDrive.roadrunner()

        waitForStart()

        drive.followTrajectorySync(drive.trajectoryBuilder().forward(Inches(60)).build())
    }
}