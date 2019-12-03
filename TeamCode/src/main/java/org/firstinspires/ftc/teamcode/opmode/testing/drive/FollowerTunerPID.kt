package org.firstinspires.ftc.teamcode.opmode.testing.drive

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.hardware.drive.brakeOnZeroPower
import org.firstinspires.ftc.teamcode.hardware.drive.enableEncoders
import org.firstinspires.ftc.teamcode.hardware.provider.makeMarkIDrive
import org.firstinspires.ftc.teamcode.util.roadrunner.RobotPosition
import org.firstinspires.ftc.teamcode.util.roadrunner.forward
import org.firstinspires.ftc.teamcode.util.roadrunner.roadrunner
import org.firstinspires.ftc.teamcode.util.units.*


/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Config
@Autonomous(group = "drive")
class FollowerPIDTuner : LinearOpMode() {

    @Override
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val drive = makeMarkIDrive(hardwareMap).also { it.brakeOnZeroPower(); it.enableEncoders() }.roadrunner()

        drive.poseEstimate = RobotPosition(-DISTANCE / 2, -DISTANCE / 2, AnglePoint.zero()).roadrunner()

        waitForStart()

        if (isStopRequested()) return

        while (!isStopRequested()) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(DISTANCE)
                            .build()
            )
            drive.turnSync(Degrees(90.0))
        }
    }

    companion object {
        var DISTANCE = Inches(28.0)
    }
}