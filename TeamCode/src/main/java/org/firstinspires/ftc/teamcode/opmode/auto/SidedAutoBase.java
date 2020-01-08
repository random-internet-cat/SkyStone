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

    protected SidedAutoBase(SideYSign ySign) {
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

    protected abstract double headingTowardsFoundationWall();
    protected abstract double headingTowardsHomeWall();

    private double headingAwayFromHomeWall() {
        return oppositeHeading(headingTowardsHomeWall());
    }

    private double headingTowardsDepotWall() {
        return oppositeHeading(headingTowardsFoundationWall());
    }

    private String formatPose(Pose2d pose) {
        return pose.toString();
    }

    private void splineToInternal(RRMecanumDriveBase drive, boolean isReversed, Pose2d pose) {
        log((isReversed ? "Reverse" : "Forward") + "-splining to: " + formatPose(pose));
        drive.followTrajectorySync(drive.trajectoryBuilder().setReversed(isReversed).splineTo(pose).build());
    }

    private void splineToReversed(RRMecanumDriveBase drive, Pose2d pose) {
        splineToInternal(drive, true, pose);
    }

    private void splineToForward(RRMecanumDriveBase drive, Pose2d pose) {
        splineToInternal(drive, false, pose);
    }

    private double grabStoneHeading() {
        return headingAwayFromHomeWall();
    }

    @Override
    protected final Pose2d startPosition() {
        return new Pose2d(sidedInchesVector(-32, 64), headingAwayFromHomeWall());
    }

    public static double GRAB_STONE_Y_POS_IN = 33;

    private double grabStoneYPos() {
        return ySidedInches(GRAB_STONE_Y_POS_IN);
    }

    @Override
    protected final void moveToGrabStoneInternal(RRMecanumDriveBase drive, QuarryState quarryState) {
        splineToForward(drive, new Pose2d(quarryState.xPosition(), grabStoneYPos(), grabStoneHeading()));
    }

    public static double MIDDLE_STOP_X_IN = 12;
    public static double MIDDLE_STOP_Y_IN = 42;

    private Vector2d middleStopPosition() {
        return sidedInchesVector(MIDDLE_STOP_X_IN, MIDDLE_STOP_Y_IN);
    }

    public static double GRAB_FOUNDATION_X_IN = 48;
    public static double GRAB_FOUNDATION_Y_IN = 31;

    private Vector2d grabFoundationPosition() {
        return sidedInchesVector(GRAB_FOUNDATION_X_IN, GRAB_FOUNDATION_Y_IN);
    }

    @Override
    protected final void moveToGrabFoundation(RRMecanumDriveBase drive) {
        drive.followTrajectorySync(drive.trajectoryBuilder()
                                        .setReversed(true)
                                        .splineTo(new Pose2d(middleStopPosition(), headingTowardsFoundationWall()))
                                        .splineTo(new Pose2d(grabFoundationPosition(), headingTowardsHomeWall()))
                                        .build());
    }

    @Override
    protected final void moveFoundationToDepot(RRMecanumDriveBase drive) {
        // Spline to make foundation horizontal
        splineToForward(drive, new Pose2d(sidedInchesVector(30, 42), headingTowardsDepotWall()));

        log("Reversing");
        drive.followTrajectorySync(drive.trajectoryBuilder()
                                        .setReversed(true)
                                        .forward(inches(-16))
                                        .build());
    }

    public static double PARK_X_IN = 0;
    public static double PARK_Y_IN = 36;

    private Vector2d parkPosition() {
        return sidedInchesVector(PARK_X_IN, PARK_Y_IN);
    }

    @Override
    protected final void park(RRMecanumDriveBase drive) {
        splineToForward(drive, new Pose2d(parkPosition(), headingTowardsDepotWall()));
    }
}
