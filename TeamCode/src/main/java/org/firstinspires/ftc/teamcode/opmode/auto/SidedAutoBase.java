package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.mecanum.RRMecanumDriveBase;

public abstract class SidedAutoBase extends AutoBase {
    protected abstract double grabStoneYPos();
    protected abstract double grabStoneHeading();

    @Override
    protected final void moveToGrabStoneInternal(RRMecanumDriveBase drive, QuarryState quarryState) {
        drive.followTrajectorySync(drive.trajectoryBuilder()
                                        .setReversed(false)
                                        .splineTo(new Pose2d(
                                            quarryState.xPosition(),
                                            grabStoneYPos(),
                                            grabStoneHeading()
                                        )).build());
    }
}
