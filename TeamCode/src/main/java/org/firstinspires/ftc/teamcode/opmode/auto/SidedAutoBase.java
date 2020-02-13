package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;

import org.firstinspires.ftc.teamcode.drive.mecanum.RRMecanumDriveBase;
import org.firstinspires.ftc.teamcode.hardware.MarkIHardware;
import org.firstinspires.ftc.teamcode.hardware.arm.MarkIArm;
import org.firstinspires.ftc.teamcode.hardware.auto_claw.MarkIAutoClaws;
import org.firstinspires.ftc.teamcode.hardware.foundation_mover.MarkIFoundationMover;
import org.firstinspires.ftc.teamcode.hardware.intake.MarkIIntake;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

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

    public static int FORCED_QUARRY_STATE = -1;

    @Override
    protected QuarryState readQuarryState() {
        if (FORCED_QUARRY_STATE == -1) {
            return mapQuarryState(readQuarryRelative());
        } else {
            return QuarryState.values()[FORCED_QUARRY_STATE];
        }
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

    private void splineToInternal(RRMecanumDriveBase drive, boolean isReversed, Pose2d pose, HeadingInterpolator interpolator) {
        log((isReversed ? "Reverse" : "Forward") + "-splining to: " + formatPose(pose));
        drive.followTrajectorySync(drive.trajectoryBuilder().setReversed(isReversed).splineTo(pose, interpolator).build());
    }

    private void splineToInternalConstantHeading(RRMecanumDriveBase drive, boolean isReversed, Pose2d pose, double heading) {
        splineToInternal(drive, isReversed, pose, new ConstantInterpolator(heading));
    }

    private void splineToReversed(RRMecanumDriveBase drive, Pose2d pose) {
        splineToInternal(drive, true, pose);
    }

    private void splineToForward(RRMecanumDriveBase drive, Pose2d pose) {
        splineToInternal(drive, false, pose);
    }

    private void splineToForwardConstantHeading(RRMecanumDriveBase drive, Pose2d pose, double heading) {
        splineToInternalConstantHeading(drive, false, pose, heading);
    }

    private void splineToReversedConstantHeading(RRMecanumDriveBase drive, Pose2d pose, double heading) {
        splineToInternalConstantHeading(drive, true, pose, heading);
    }

    private void turnToHeading(RRMecanumDriveBase drive, double heading) {
        log("Turning to: " + (heading * 180/PI));
        drive.turnSync(sub(ofHeading(heading), ofHeading(drive.getPoseEstimate().getHeading())));
    }

    protected abstract double grabStoneHeading();

    @Override
    protected final Pose2d startPosition() {
        return new Pose2d(sidedInchesVector(-36, 65.5), headingAwayFromHomeWall());
    }

    @Override
    protected void turnTowardsWall(RRMecanumDriveBase drive) {
        drive.followTrajectorySync(drive.trajectoryBuilder().forward(inches(6)).build());
        turnToHeading(drive, headingTowardsHomeWall());
    }

    public static double GRAB_STONE_Y_POS_IN = 32;

    private double grabStoneYPos() {
        return ySidedInches(GRAB_STONE_Y_POS_IN);
    }

    public static double STONE_GRAB_OFFSET_IN = 0;

    public static double STONE_CURVE_OFFSET_IN = 7;

    protected abstract double grabStoneCurveHeading();

    private Pose2d firstStoneGrabCurvePosition(QuarryState quarryState) {
        double heading = grabStoneCurveHeading();
        double headingDiff = heading - headingTowardsDepotWall();
        return new Pose2d(quarryState.firstXPosition() + Math.cos(headingDiff) * inches(STONE_CURVE_OFFSET_IN), grabStoneYPos() + Math.sin(headingDiff) * inches(STONE_CURVE_OFFSET_IN), grabStoneCurveHeading());
    }

    protected Pose2d firstStoneGrabPosition(QuarryState quarryState) {
        return new Pose2d(
            quarryState.firstXPosition() + inches(STONE_GRAB_OFFSET_IN),
            grabStoneYPos(),
            grabStoneHeading()
        );
    }

    public static double RELEASE_STONE_X_IN = 57;
    public static double RELEASE_STONE_Y_IN = 34;

    private Pose2d releaseFirstStonePosition(QuarryState quarryState) {
        return new Pose2d(
            sidedInchesVector(RELEASE_STONE_X_IN, RELEASE_STONE_Y_IN),
            headingTowardsFoundationWall()
        );
    }

    public static double RELEASE_FIRST_STONE_MIDDLE_STOP_X = 0;
    public static double RELEASE_FIRST_STONE_MIDDLE_STOP_Y = 40;

    private Pose2d releaseFirstStoneMiddleStopPosition(QuarryState quarryState) {
        return new Pose2d(
            sidedInchesVector(RELEASE_FIRST_STONE_MIDDLE_STOP_X, RELEASE_FIRST_STONE_MIDDLE_STOP_Y),
            headingTowardsDepotWall()
        );
    }

    public static double RELEASE_FIRST_STONE_PREMIDDLE_STOP_X = -12;
    public static double RELEASE_FIRST_STONE_PREMIDDLE_STOP_Y = 40;

    private Pose2d releaseFirstStonePreMiddleStopPosition(QuarryState quarryState) {
        return new Pose2d(
            sidedInchesVector(RELEASE_FIRST_STONE_PREMIDDLE_STOP_X, RELEASE_FIRST_STONE_PREMIDDLE_STOP_Y),
            headingTowardsDepotWall()
        );
    }

    private Pose2d releaseSecondStonePosition(QuarryState quarryState) {
        return new Pose2d(
            sidedInchesVector(RELEASE_STONE_X_IN, RELEASE_STONE_Y_IN),
            headingTowardsDepotWall()
        );
    }

    private void moveToReleaseSecondStone(RRMecanumDriveBase drive, QuarryState quarryState) {
        splineToReversed(drive, releaseSecondStonePosition(quarryState));
    }

    private void moveToFirstStone(RRMecanumDriveBase drive, QuarryState quarryState) {
        drive.followTrajectorySync(drive.trajectoryBuilder()
                                        .setReversed(false)
                                        .splineTo(firstStoneGrabCurvePosition(quarryState))
                                        .splineTo(firstStoneGrabPosition(quarryState))
                                        .build());
    }

    @Override
    protected void handleFirstStone(MarkIHardware hardware, QuarryState quarryState) throws InterruptedException {
        RRMecanumDriveBase drive = hardware.getDrive().roadrunner();
        MarkIIntake intake = hardware.getIntake();

        intake.intake();

        log("Moving to grab first stone");
        moveToFirstStone(drive, quarryState);
        log("Moved to grab first stone");

        checkInterrupted();

        sleep(750 /* ms */);
        intake.stop();
        hardware.getArm().getClamp().close();
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

    @Override
    protected void handleSecondStone(MarkIHardware hardware, QuarryState quarryState) throws InterruptedException {
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
        //log("Moving to release second stone");
        //moveToReleaseSecondStone(drive, quarryState);
        //log("Moved to release second stone");
        //
        //log("Releasing stone");
        //releaseStone(hardware);
        //log("Released stone");
    }

    protected abstract void prepareToGrabStone(MarkIHardware hardware);

    //protected abstract void collectSecondStone(MarkIHardware hardware, QuarryState quarryState);

    public static double GRAB_FOUNDATION_X_IN = 41.5;
    public static double GRAB_FOUNDATION_Y_IN = 27;

    private Vector2d grabFoundationPosition() {
        return sidedInchesVector(GRAB_FOUNDATION_X_IN, GRAB_FOUNDATION_Y_IN);
    }

    @Override
    protected final void grabFoundation(MarkIHardware hardware) {
        MarkIArm arm = hardware.getArm();
        MarkIFoundationMover mover = hardware.getFoundationMover();

        log("Dropping stone and settings movers to grab");
        arm.getClamp().open();
        mover.grab();

        sleep( 400 /* ms */);

        log("Stone dropped & movers in grab position");

        log("Retracting horizontal");
        arm.getHorizontal().moveAllTheWayIn();
    }

    @Override
    protected final void moveToGrabFoundation(RRMecanumDriveBase drive, QuarryState quarryState, final MarkIArm arm) {
        drive.followTrajectorySync(drive.trajectoryBuilder()
                                        .setReversed(true)
                                        //.splineTo(releaseFirstStonePreMiddleStopPosition(quarryState))
                                        .splineTo(releaseFirstStoneMiddleStopPosition(quarryState))
                                        .addMarker(2 /*seconds*/, new Function0<Unit>() {
                                            @Override
                                            public Unit invoke() {
                                                arm.getClamp().close();
                                                arm.getVertical().moveToPlace(1);
                                                arm.getHorizontal().moveAllTheWayOut();
                                                return Unit.INSTANCE;
                                            }
                                        })
                                        .splineTo(new Pose2d(grabFoundationPosition(), headingTowardsHomeWall()))
                                        .build());
    }

    public static double MOVE_FOUNDATION_TO_BUILDING_ZONE_REVERSE_DISTANCE_IN = 12;
    public static double ALIGN_FOUNDATION_X_IN = 28;
    public static double ALIGN_FOUNDATION_Y_IN = 51;

    private Pose2d alignFoundationPosition() {
        return new Pose2d(sidedInchesVector(ALIGN_FOUNDATION_X_IN, ALIGN_FOUNDATION_Y_IN), headingTowardsDepotWall());
    }

    @Override
    protected final void moveFoundationToBuildingZoneAndRetractArm(RRMecanumDriveBase drive, final MarkIArm arm) {
        // Spline to make foundation horizontal
        splineToForward(drive, alignFoundationPosition());

        log("Reversing");
        drive.followTrajectorySync(drive.trajectoryBuilder()
                                        .setReversed(true)
                                        .forward(inches(-MOVE_FOUNDATION_TO_BUILDING_ZONE_REVERSE_DISTANCE_IN))
                                        .addMarker(0.5 /*seconds*/, new Function0<Unit>() {
                                            @Override
                                            public Unit invoke() {
                                                arm.getVertical().moveToCollect();
                                                return Unit.INSTANCE;
                                            }
                                        })
                                        .build());
    }

    public static double PARK_X_IN = 0;
    public static double PARK_Y_IN = 47;

    private Vector2d parkPosition() {
        return sidedInchesVector(PARK_X_IN, PARK_Y_IN);
    }

    @Override
    protected final void park(RRMecanumDriveBase drive) {
        splineToForward(drive, new Pose2d(parkPosition(), headingTowardsDepotWall()));
    }
}
