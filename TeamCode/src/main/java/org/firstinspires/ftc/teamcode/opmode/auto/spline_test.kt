package org.firstinspires.ftc.teamcode.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.hardware.provider.makeMarkIHardware
import org.firstinspires.ftc.teamcode.util.roadrunner.RobotPosition
import org.firstinspires.ftc.teamcode.util.roadrunner.splineTo
import org.firstinspires.ftc.teamcode.util.units.*

@Autonomous
class SplineTest : LinearOpMode() {
    override fun runOpMode() {
        val hardware = makeMarkIHardware(hardwareMap)
        val drive = hardware.drive.roadrunner()

        waitForStart()

        drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(RobotPosition(
            x = Inches(30),
            y = Inches(30),
            heading = AnglePoint.zero()
        )).build())

        sleep(2000)

        drive.followTrajectorySync(drive.trajectoryBuilder().reverse().splineTo(RobotPosition(
            x = Distance.zero(),
            y = Distance.zero(),
            heading = AnglePoint.zero()
        )).build())
    }
}