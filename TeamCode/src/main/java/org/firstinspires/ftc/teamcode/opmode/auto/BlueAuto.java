package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.mecanum.RRMecanumDriveBase;

import java.util.Objects;

import static org.firstinspires.ftc.teamcode.util.RRUnits.degHeading;
import static org.firstinspires.ftc.teamcode.util.RRUnits.inches;
import static org.firstinspires.ftc.teamcode.util.RRUnits.inchesVector;

@Autonomous
@Config
public class BlueAuto extends SidedAutoBase {
    BlueAuto() {
        super(SideYSign.POSITIVE);
    }

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
    protected double headingTowardsFoundationWall() {
        return degHeading(180);
    }

    @Override
    protected double headingTowardsHomeWall() {
        return degHeading(90);
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
                                        .splineTo(new Pose2d(inchesVector(0, 36), degHeading(0)))
                                        .build());
    }
}
