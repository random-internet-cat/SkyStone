package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.mecanum.RRMecanumDriveBase;
import org.firstinspires.ftc.teamcode.hardware.MarkIHardware;
import org.firstinspires.ftc.teamcode.hardware.foundation_mover.MarkIFoundationMover;
import org.firstinspires.ftc.teamcode.hardware.rear_claw.MarkIRearClaws;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.util.RRUnits.inches;
import static org.firstinspires.ftc.teamcode.util.RRUnits.inchesVector;
import static org.firstinspires.ftc.teamcode.util.RRUnits.ofHeading;
import static org.firstinspires.ftc.teamcode.util.RRUnits.oppositeHeading;
import static org.firstinspires.ftc.teamcode.util.RRUnits.sub;

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

    enum SideColor {
        RED(SideYSign.POSITIVE),
        BLUE(SideYSign.NEGATIVE)
        ;

        private final SideYSign sideYSign;

        private SideColor(SideYSign sideYSign) {
            this.sideYSign = sideYSign;
        }

        public SideYSign sideYSign() { return sideYSign; }
    }

    private final SideColor sideColor;

    protected SidedAutoBase(SideColor sideColor) {
        this.sideColor = sideColor;
    }

    private double sideY(double raw) {
        return sideColor.sideYSign().scale(raw);
    }

    private double ySidedInches(double y) {
        return sideY(inches(y));
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

    private void turnToHeading(RRMecanumDriveBase drive, double heading) {
        log("Turning to: " + (heading * 180/PI));
        drive.turnSync(sub(ofHeading(heading), ofHeading(drive.getPoseEstimate().getHeading())));
    }

    private double grabStoneHeading() {
        return headingTowardsHomeWall();
    }

    @Override
    protected final Pose2d startPosition() {
        return new Pose2d(sidedInchesVector(-36, 64), headingAwayFromHomeWall());
    }

    @Override
    protected void turnTowardsWall(RRMecanumDriveBase drive) {
        drive.followTrajectorySync(drive.trajectoryBuilder().forward(inches(6)).build());
        turnToHeading(drive, headingTowardsHomeWall());
    }

    public static double GRAB_STONE_Y_POS_IN = 36;

    private double grabStoneYPos() {
        return ySidedInches(GRAB_STONE_Y_POS_IN);
    }

    protected abstract void moveFoundationMoverToCollect(MarkIFoundationMover foundationMover);

    @Override
    protected void prepareToGrabStone(MarkIHardware hardware, QuarryState quarryState) {
        moveFoundationMoverToCollect(hardware.getFoundationMover());
        hardware.getRearClaws().releaseBoth();
    }

    @Override
    protected final void moveToGrabStone(RRMecanumDriveBase drive, QuarryState quarryState) {
        splineToReversed(drive, new Pose2d(quarryState.xPosition(), grabStoneYPos(), grabStoneHeading()));
    }

    protected abstract void clampRearClawsForStone(MarkIRearClaws claws);
    protected abstract void releaseRearClawsForStone(MarkIRearClaws claws);

    protected abstract void moveStoneAboveGround(MarkIFoundationMover foundationMover);

    @Override
    protected void grabStone(MarkIHardware hardware, QuarryState quarryState) {
        clampRearClawsForStone(hardware.getRearClaws());
        sleep(500);
        moveStoneAboveGround(hardware.getFoundationMover());
        sleep(500);
    }

    public static double MIDDLE_STOP_X_IN = 12;
    public static double MIDDLE_STOP_Y_IN = 43;

    private Vector2d middleStopPosition() {
        return sidedInchesVector(MIDDLE_STOP_X_IN, MIDDLE_STOP_Y_IN);
    }

    public static double GRAB_FOUNDATION_X_IN = 52;
    public static double GRAB_FOUNDATION_Y_IN = 51;

    private Vector2d grabFoundationPosition() {
        return sidedInchesVector(GRAB_FOUNDATION_X_IN, GRAB_FOUNDATION_Y_IN);
    }

    public static double GRAB_FOUNDATION_REVERSE_DISTANCE_IN = 16;

    @Override
    protected final void prepareToGrabFoundation(MarkIHardware hardware) {
        hardware.getFoundationMover().moveStoneAboveGround();
    }

    @Override
    protected final void moveToGrabFoundation(RRMecanumDriveBase drive) {
        drive.followTrajectorySync(drive.trajectoryBuilder()
                                        .setReversed(false)
                                        .splineTo(new Pose2d(middleStopPosition(), headingTowardsFoundationWall()))
                                        .splineTo(new Pose2d(grabFoundationPosition(), headingTowardsHomeWall()))
                                        .build());

        log("Reversing");
        drive.followTrajectorySync(drive.trajectoryBuilder()
                                        .setReversed(true)
                                        .forward(inches(-GRAB_FOUNDATION_REVERSE_DISTANCE_IN))
                                        .build());
    }

    public static int KICK_STONE_FALL_SLEEP_MS = 100;
    public static int KICK_STONE_MOVE_SLEEP_MS = 500;

    @Override
    protected void releaseStone(MarkIHardware hardware, QuarryState quarryState) {
        releaseRearClawsForStone(hardware.getRearClaws());
        sleep(KICK_STONE_FALL_SLEEP_MS);
        hardware.getFoundationMover().release();
        sleep(KICK_STONE_MOVE_SLEEP_MS);
    }

    public static double MOVE_FOUNDATION_TO_BUILDING_ZONE_REVERSE_DISTANCE_IN = 16;
    public static double ALIGN_FOUNDATION_X_IN = 27;
    public static double ALIGN_FOUNDATION_Y_IN = 45;

    private Pose2d alignFoundationPosition() {
        return new Pose2d(sidedInchesVector(ALIGN_FOUNDATION_X_IN, ALIGN_FOUNDATION_Y_IN), headingTowardsDepotWall());
    }

    @Override
    protected final void moveFoundationToBuildingZone(RRMecanumDriveBase drive) {
        // Spline to make foundation horizontal
        splineToForward(drive, alignFoundationPosition());

        log("Reversing");
        drive.followTrajectorySync(drive.trajectoryBuilder()
                                        .setReversed(true)
                                        .forward(inches(-MOVE_FOUNDATION_TO_BUILDING_ZONE_REVERSE_DISTANCE_IN))
                                        .build());
    }

    public static double PARK_X_IN = 0;
    public static double PARK_Y_IN = 44;

    private Vector2d parkPosition() {
        return sidedInchesVector(PARK_X_IN, PARK_Y_IN);
    }

    @Override
    protected final void park(RRMecanumDriveBase drive) {
        splineToForward(drive, new Pose2d(parkPosition(), headingTowardsDepotWall()));
    }
}
