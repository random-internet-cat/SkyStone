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
        RED(SideYSign.NEGATIVE),
        BLUE(SideYSign.POSITIVE)
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

    protected final SkystoneRelativePos readQuarryRelative() {
        SkystoneDetector detector = new SkystoneDetector();
        return detector.getPosition(hardwareMap, telemetry, /*nearBlueTape=*/ sideColor == SideColor.BLUE);
    }

    protected abstract QuarryState mapQuarryState(SkystoneRelativePos relativePos);

    @Override
    protected QuarryState readQuarryState() {
        return mapQuarryState(readQuarryRelative());
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
        return headingAwayFromHomeWall();
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

    public static double STONE_GRAB_OFFSET_IN = 0;

    protected Pose2d firstStoneGrabPosition(QuarryState quarryState) {
        return new Pose2d(
            quarryState.firstXPosition() + inches(STONE_GRAB_OFFSET_IN),
            grabStoneYPos(),
            grabStoneHeading()
        );
    }

    private void prepareToGrabStone(MarkIHardware hardware) {
        moveFoundationMoverToCollect(hardware.getFoundationMover());
        hardware.getRearClaws().releaseBoth();
    }

    public static double RELEASE_STONE_X_IN = 20;
    public static double RELEASE_STONE_Y_IN = 43;

    private Pose2d releaseFirstStonePosition(QuarryState quarryState) {
        return new Pose2d(
            sidedInchesVector(RELEASE_STONE_X_IN, RELEASE_STONE_Y_IN),
            headingTowardsHomeWall()
        );
    }

    private Pose2d releaseSecondStonePosition(QuarryState quarryState) {
        return new Pose2d(
            sidedInchesVector(RELEASE_STONE_X_IN, RELEASE_STONE_Y_IN),
            headingTowardsHomeWall()
        );
    }

    private void moveToReleaseFirstStone(RRMecanumDriveBase drive, QuarryState quarryState) {
        splineToReversed(drive, releaseFirstStonePosition(quarryState));
    }

    private void moveToReleaseSecondStone(RRMecanumDriveBase drive, QuarryState quarryState) {
        splineToReversed(drive, releaseSecondStonePosition(quarryState));
    }

    private void releaseStone(MarkIHardware hardware) {
        releaseRearClawsForStone(hardware.getRearClaws());
    }

    @Override
    protected void handleFirstStone(MarkIHardware hardware, RRMecanumDriveBase drive, QuarryState quarryState) throws InterruptedException {
        log("Preparing to grab first stone");
        prepareToGrabStone(hardware);
        log("Prepared to grab first stone");

        checkInterrupted();

        log("Moving to grab first stone");
        splineToForward(drive, firstStoneGrabPosition(quarryState));
        log("Moved to grab first stone");

        checkInterrupted();

        log("Collecting first stone");
        collectFirstStone(hardware, quarryState);
        log("Collected first stone");

        checkInterrupted();

        log("Moving first stone above ground");
        moveStoneAboveGround(hardware.getFoundationMover());
        log("Moving second stone above ground");

        log("Moving to release first stone");
        moveToReleaseFirstStone(drive, quarryState);
        log("Moved to release first stone");

        log("Releasing stone");
        releaseStone(hardware);
        log("Released stone");
    }

    private Pose2d secondStoneGrabPosition(QuarryState quarryState) {
        return new Pose2d(
            quarryState.secondXPosition() + inches(STONE_GRAB_OFFSET_IN),
            grabStoneYPos(),
            grabStoneHeading()
        );
    }

    private void moveToGrabSecondStone(RRMecanumDriveBase drive, QuarryState quarryState) {
        splineToForward(drive, secondStoneGrabPosition(quarryState));
    }

    protected void handleSecondStone(MarkIHardware hardware, RRMecanumDriveBase drive, QuarryState quarryState) throws InterruptedException {
        //log("Preparing to grab second stone");
        //prepareToGrabStone(hardware);
        //log("Prepared to grab second stone");
        //
        //checkInterrupted();
        //
        //log("Moving to grab second stone");
        //moveToGrabSecondStone(drive, quarryState);
        //log("Moved to grab second stone");
        //
        //checkInterrupted();
        //
        //log("Collecting second stone");
        //collectSecondStone(hardware, quarryState);
        //log("Collected second stone");
        //
        //checkInterrupted();
        //
        //log("Moving second stone above ground");
        //moveStoneAboveGround(hardware.getFoundationMover());
        //log("Moving second stone above ground");
        //
        //log("Moving to release second stone");
        //moveToReleaseSecondStone(drive, quarryState);
        //log("Moved to release second stone");
        //
        //log("Releasing stone");
        //releaseStone(hardware);
        //log("Released stone");
    }

    protected abstract void clampRearClawsForStone(MarkIRearClaws claws);
    protected abstract void releaseRearClawsForStone(MarkIRearClaws claws);

    protected abstract void moveStoneAboveGround(MarkIFoundationMover foundationMover);

    protected void collectFirstStone(MarkIHardware hardware, QuarryState quarryState) {
        clampRearClawsForStone(hardware.getRearClaws());
    }
    //protected abstract void collectSecondStone(MarkIHardware hardware, QuarryState quarryState);

    public static double GRAB_FOUNDATION_X_IN = 52;
    public static double GRAB_FOUNDATION_Y_IN = 35;

    private Vector2d grabFoundationPosition() {
        return sidedInchesVector(GRAB_FOUNDATION_X_IN, GRAB_FOUNDATION_Y_IN);
    }

    @Override
    protected final void prepareToGrabFoundation(MarkIHardware hardware) {
    }

    @Override
    protected final void moveToGrabFoundation(RRMecanumDriveBase drive) {
        splineToReversed(drive, new Pose2d(grabFoundationPosition(), headingTowardsHomeWall()));
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
