package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.mecanum.RRMecanumDriveBase;

import java.util.Objects;

import static org.firstinspires.ftc.teamcode.util.RRUnits.degHeading;
import static org.firstinspires.ftc.teamcode.util.RRUnits.inches;
import static org.firstinspires.ftc.teamcode.util.RRUnits.inchesVector;

@Autonomous
public class BlueAuto extends AutoBase {
    @Override
    protected QuarryState mapQuarryState(SkystoneRelativePos relativePos) {
        Objects.requireNonNull(relativePos, "relativePos");

        switch (relativePos) {
            case RIGHT: return QuarryState.CLOSE_TO_WALL;
            case MIDDLE: return QuarryState.MIDDLE;
            case LEFT: return QuarryState.CLOSE_TO_BRIDGES;
        }

        throw new UnsupportedOperationException();
    }

    @Override
    protected Pose2d startPosition() {
        return new Pose2d(inchesVector(-32.0, 64.0), degHeading(270));
    }

    @Override
    protected double grabStoneYPos() {
        return inches(33);
    }

    @Override
    protected double grabStoneHeading() {
        return degHeading(180);
    }

    @Override
    protected void moveToGrabFoundation(RRMecanumDriveBase drive) {
        // Move to center of field
        drive.followTrajectorySync(drive.trajectoryBuilder()
                                        .setReversed(true)
                                        .splineTo(new Pose2d(inchesVector(12, 36), degHeading(180)))
                                        .build());

        // Move to foundation
        drive.followTrajectorySync(drive.trajectoryBuilder()
                                        .splineTo(new Pose2d(inchesVector(48, 31), degHeading(90)))
                                        .build()
        );
    }

    @Override
    protected void moveFoundationToDepot(RRMecanumDriveBase drive) {
        // Spline to make foundation horizontal
        drive.followTrajectorySync(drive.trajectoryBuilder()
                                        .setReversed(false)
                                        .splineTo(new Pose2d(inchesVector(30, 42), degHeading(180)))
                                        .build());

        drive.followTrajectorySync(drive.trajectoryBuilder()
                                        .setReversed(true)
                                        .forward(inches(-16))
                                        .build());
    }

    @Override
    protected void park(RRMecanumDriveBase drive) {
        drive.followTrajectorySync(drive.trajectoryBuilder()
                                        .setReversed(false)
                                        .splineTo(new Pose2d(inchesVector(0, 36), degHeading(180)))
                                        .build());
    }
}
