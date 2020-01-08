package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.mecanum.RRMecanumDriveBase;

import static org.firstinspires.ftc.teamcode.util.RRUnits.inches;
import static org.firstinspires.ftc.teamcode.util.RRUnits.inchesVector;
import static org.firstinspires.ftc.teamcode.util.RRUnits.oppositeHeading;

@Config
public abstract class SidedAutoBase extends AutoBase {
    enum SideYSign {
        POSITIVE(1),
        NEGATIVE(-1)
        ;

        private final int multiplier;

        private SideYSign(int multiplier) { this.multiplier = multiplier; }

        double scale(double unsignedY) {
            return unsignedY * multiplier;
        }
    }

    private final SideYSign ySign;

    SidedAutoBase(SideYSign ySign) {
        this.ySign = ySign;
    }

    private double sideY(double raw) {
        return ySign.scale(raw);
    }

    private double ySidedInches(double y) {
        return inches(y);
    }

    private Vector2d sidedInchesVector(double x, double y) {
        return inchesVector(x, sideY(y));
    }

    public static double GRAB_STONE_Y_POS_IN = 33;

    private double grabStoneYPos() {
        return ySidedInches(GRAB_STONE_Y_POS_IN);
    }

    protected abstract double headingTowardsFoundationWall();
    protected abstract double headingTowardsHomeWall();

    private double headingAwayFromHomeWall() {
        return oppositeHeading(headingTowardsHomeWall());
    }

    private double headingTowardsDepotWall() {
        return oppositeHeading(headingTowardsFoundationWall());
    }

    private double grabStoneHeading() {
        return headingAwayFromHomeWall();
    }

    @Override
    protected final Pose2d startPosition() {
        return new Pose2d(sidedInchesVector(-32, 64), headingAwayFromHomeWall());
    }

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

    @Override
    protected final void moveToGrabFoundation(RRMecanumDriveBase drive) {
        // Move to center of field
        drive.followTrajectorySync(drive.trajectoryBuilder()
                                        .setReversed(true)
                                        .splineTo(new Pose2d(sidedInchesVector(12, 42), headingTowardsFoundationWall()))
                                        .build());

        // Move to foundation
        drive.followTrajectorySync(drive.trajectoryBuilder()
                                        .setReversed(true)
                                        .splineTo(new Pose2d(sidedInchesVector(48, 31), headingTowardsHomeWall()))
                                        .build()
        );
    }

    @Override
    protected final void moveFoundationToDepot(RRMecanumDriveBase drive) {
        // Spline to make foundation horizontal
        drive.followTrajectorySync(drive.trajectoryBuilder()
                                        .setReversed(false)
                                        .splineTo(new Pose2d(sidedInchesVector(30, 42), headingTowardsDepotWall()))
                                        .build());

        drive.followTrajectorySync(drive.trajectoryBuilder()
                                        .setReversed(true)
                                        .forward(inches(-16))
                                        .build());
    }
}
