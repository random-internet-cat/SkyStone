package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.mecanum.RRMecanumDriveBase;
import org.firstinspires.ftc.teamcode.hardware.MarkIHardware;
import org.firstinspires.ftc.teamcode.hardware.arm.MarkIArm;
import org.firstinspires.ftc.teamcode.hardware.foundation_mover.MarkIFoundationMover;
import org.firstinspires.ftc.teamcode.hardware.intake.MarkIIntake;

import java.io.File;

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
    private final DriveConstraints FAST_CONSTRAINTS = AutoConstraintsKt.AUTO_FAST_CONSTRAINTS;

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

    @Override
    protected SkystoneDetector setupDetector() {
        return new SkystoneDetector(hardwareMap, telemetry, sideColor == SideColor.BLUE);
    }

    protected final SkystoneRelativePos readQuarryRelative(SkystoneDetector detector) {
        return detector.getPosition();
    }

    protected abstract QuarryState mapQuarryState(SkystoneRelativePos relativePos);

    public static int FORCED_QUARRY_STATE = -1;

    @Override
    protected QuarryState readQuarryState(SkystoneDetector detector) {
        if (FORCED_QUARRY_STATE == -1) {
            return mapQuarryState(readQuarryRelative(detector));
        } else {
            return QuarryState.values()[FORCED_QUARRY_STATE];
        }
    }

    protected abstract double headingTowardsFoundationWall();
    protected abstract double headingTowardsHomeWall();

    @Override
    protected final void saveGyroDataSided(RRMecanumDriveBase drive) {
        File gyroAutoHeadingFile = AppUtil.getInstance().getSettingsFile("gyroAutoHeading.txt");
        ReadWriteFile.writeFile(gyroAutoHeadingFile, String.valueOf(drive.getLocalizer().getPoseEstimate().getHeading()));
    }

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

    protected abstract double grabFirstStoneHeading();
    protected abstract double grabSecondStoneHeading(QuarryState quarryState);

    @Override
    protected final Pose2d startPosition() {
        return new Pose2d(sidedInchesVector(-36, 65.5), headingAwayFromHomeWall());
    }

    @Override
    protected void turnTowardsWall(RRMecanumDriveBase drive) {
        drive.followTrajectorySync(drive.trajectoryBuilder().forward(inches(6)).build());
        turnToHeading(drive, headingTowardsHomeWall());
    }

    public static double GRAB_FIRST_STONE_Y_POS_IN = 30;

    private double grabFirstStoneYPos() {
        return ySidedInches(GRAB_FIRST_STONE_Y_POS_IN);
    }

    public static double GRAB_SECOND_STONE_Y_POS_IN = 28;

    private double grabSecondStoneYPos() {
        return ySidedInches(GRAB_SECOND_STONE_Y_POS_IN);
    }

    public static double FIRST_STONE_GRAB_OFFSET_IN = 0;

    public static double STONE_CURVE_OFFSET_IN = 5;

    private Pose2d firstStoneGrabCurvePosition(QuarryState quarryState) {
        double heading = grabFirstStoneHeading();
        double headingDiff = heading - headingTowardsDepotWall();
        return new Pose2d(quarryState.firstXPosition() + Math.cos(headingDiff) * inches(STONE_CURVE_OFFSET_IN), grabFirstStoneYPos() + Math.sin(headingDiff) * inches(STONE_CURVE_OFFSET_IN), heading);
    }

    protected Pose2d firstStoneGrabPosition(QuarryState quarryState) {
        return new Pose2d(
            quarryState.firstXPosition() + inches(FIRST_STONE_GRAB_OFFSET_IN),
            grabFirstStoneYPos(),
            grabFirstStoneHeading()
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
    public static double RELEASE_FIRST_STONE_MIDDLE_STOP_Y = 43;

    private Pose2d releaseFirstStoneMiddleStopPosition(QuarryState quarryState) {
        return new Pose2d(
            sidedInchesVector(RELEASE_FIRST_STONE_MIDDLE_STOP_X, RELEASE_FIRST_STONE_MIDDLE_STOP_Y),
            headingTowardsDepotWall()
        );
    }

    /*public static double RELEASE_FIRST_STONE_PREMIDDLE_STOP_X = -12;
    public static double RELEASE_FIRST_STONE_PREMIDDLE_STOP_Y = 40;

    private Pose2d releaseFirstStonePreMiddleStopPosition(QuarryState quarryState) {
        return new Pose2d(
            sidedInchesVector(RELEASE_FIRST_STONE_PREMIDDLE_STOP_X, RELEASE_FIRST_STONE_PREMIDDLE_STOP_Y),
            headingTowardsDepotWall()
        );
    }*/

    private Pose2d releaseSecondStonePosition(QuarryState quarryState) {
        return new Pose2d(
            sidedInchesVector(RELEASE_STONE_X_IN, RELEASE_STONE_Y_IN),
            headingTowardsDepotWall()
        );
    }

    private void moveToReleaseSecondStone(RRMecanumDriveBase drive, QuarryState quarryState) {
        splineToReversed(drive, releaseSecondStonePosition(quarryState));
    }

    private void moveToFirstStone(RRMecanumDriveBase drive, QuarryState quarryState, final MarkIIntake intake) {
        drive.followTrajectorySync(drive.trajectoryBuilder()
                                        .setReversed(false)
                                        .addMarker(1.0 /*seconds*/, new Function0<Unit>() {
                                            @Override
                                            public Unit invoke() {
                                                intake.intake();
                                                return Unit.INSTANCE;
                                            }
                                        })
                                        .splineTo(firstStoneGrabCurvePosition(quarryState))
                                        .splineTo(firstStoneGrabPosition(quarryState))
                                        .build());
    }

    @Override
    protected void handleFirstStone(MarkIHardware hardware, QuarryState quarryState) throws InterruptedException {
        RRMecanumDriveBase drive = hardware.getDrive().roadrunner();
        MarkIIntake intake = hardware.getIntake();

        log("Moving to grab first stone");
        moveToFirstStone(drive, quarryState, intake);
        log("Moved to grab first stone");

        checkInterrupted();
    }

    private Pose2d secondStoneGrabPosition(QuarryState quarryState) {
        return new Pose2d(
            quarryState.secondXPosition(),
            grabSecondStoneYPos(),
            grabSecondStoneHeading(quarryState)
        );
    }

    private Pose2d secondStoneMidpointPosition() {
        return new Pose2d(
            skybridgeMiddlePosition(),
            headingTowardsDepotWall()
        );
    }

    private Pose2d secondStoneBeforeGrabPosition(QuarryState quarryState) {
        double heading = grabSecondStoneHeading(quarryState);
        double headingDiff = heading - headingTowardsDepotWall();
        double curveOffset = inches(STONE_CURVE_OFFSET_IN);

        return new Pose2d(
            quarryState.secondXPosition() + Math.cos(headingDiff) * curveOffset,
            grabSecondStoneYPos() + Math.sin(headingDiff) * curveOffset,
            heading
        );
    }

    // Now also turns on the intake, saves 3 seconds of intake actuation (can drain battery significantly)
    private void moveToGrabSecondStone(RRMecanumDriveBase drive, final MarkIIntake intake, QuarryState quarryState) {
        drive.followTrajectorySync(drive.trajectoryBuilder()
                                        .splineTo(secondStoneMidpointPosition())
                                        .addMarker(3.0 /* seconds */, new Function0<Unit>() {
                                            @Override
                                            public Unit invoke() {
                                                intake.intake();
                                                return Unit.INSTANCE;
                                            }
                                        })
                                        .splineTo(secondStoneBeforeGrabPosition(quarryState))
                                        .splineTo(secondStoneGrabPosition(quarryState))
                                        .build());
    }

    private Pose2d depositSecondStoneMidpointPosition() {
        return new Pose2d(
            skybridgeMiddlePosition(),
            headingTowardsDepotWall()
        );
    }

    public static double DEPOSIT_SECOND_STONE_X_IN = 38;
    public static double DEPOSIT_SECOND_STONE_Y_IN = 56;

    private Pose2d depositSecondStonePosition() {
        return new Pose2d(sidedInchesVector(DEPOSIT_SECOND_STONE_X_IN, DEPOSIT_SECOND_STONE_Y_IN), headingTowardsDepotWall());
    }

    private void moveToDepositSecondStone(final MarkIHardware hardware) {
        RRMecanumDriveBase drive = hardware.getDrive().roadrunner();
        final MarkIArm arm = hardware.getArm();

        Pose2d middleStopPos = depositSecondStoneMidpointPosition();

        Vector2d startArmMovementPos = new Vector2d(
            middleStopPos.getX() + inches(12),
            middleStopPos.getY()
        );

        drive.followTrajectorySync(drive.trajectoryBuilder()
                                        .setReversed(true)
                                        .splineTo(middleStopPos)
                                        .splineTo(depositSecondStonePosition())
                                        .addMarker(1.5 /* seconds */, new Function0<Unit>() {
                                            @Override
                                            public Unit invoke() {
                                                hardware.getIntake().stop();
                                                arm.getClamp().close();
                                                return Unit.INSTANCE;
                                            }
                                        })
                                        .addMarker(startArmMovementPos, new Function0<Unit>() {
                                            @Override
                                            public Unit invoke() {
                                                arm.getClamp().close();
                                                arm.getVertical().moveToPlace(1);
                                                arm.getHorizontal().moveAllTheWayOut();
                                                return Unit.INSTANCE;
                                            }
                                        })
                                        .build());
    }

    private void depositSecondStone(MarkIHardware hardware) throws InterruptedException {
        MarkIArm.VerticalControl vertical = hardware.getArm().getVertical();
        MarkIArm.Clamp clamp = hardware.getArm().getClamp();

        vertical.moveToCollect();
        sleep(300);

        clamp.open();
        sleep(300);

        vertical.moveToPlace(1);
        sleep(300);
    }

    @Override
    protected void handleSecondStone(MarkIHardware hardware, QuarryState quarryState) throws InterruptedException {
        RRMecanumDriveBase drive = hardware.getDrive().roadrunner();
        MarkIArm arm = hardware.getArm();
        MarkIIntake intake = hardware.getIntake();

        checkInterrupted();

        log("Moving to grab second stone");
        moveToGrabSecondStone(drive, intake, quarryState);
        log("Moved to grab second stone");

        checkInterrupted();

        log("Moving to deposit second stone");
        moveToDepositSecondStone(hardware);
        log("Moved to deposit second stone");

        checkInterrupted();

        log("Releasing second stone");
        arm.getVertical().moveToCollect();
        sleep(300);

        openClampAndWait(arm.getClamp());

        arm.getVertical().moveToPlace(1);
        sleep(300);
        log("Released second stone");

        log("Retracting horizontal");

        arm.getHorizontal().moveAllTheWayIn();
    }

    protected abstract void prepareToGrabStone(MarkIHardware hardware);

    //protected abstract void collectSecondStone(MarkIHardware hardware, QuarryState quarryState);

    public static double GRAB_FOUNDATION_X_IN = 42.5;
    public static double GRAB_FOUNDATION_Y_IN = 31.5;

    private Vector2d grabFoundationPosition() {
        return sidedInchesVector(GRAB_FOUNDATION_X_IN, GRAB_FOUNDATION_Y_IN);
    }

    @Override
    protected final void grabFoundation(MarkIHardware hardware) {
        MarkIArm arm = hardware.getArm();
        MarkIFoundationMover mover = hardware.getFoundationMover();

        log("Dropping stone and settings movers to grab");
        arm.getVertical().moveToCollect();
        sleep(300);

        arm.getClamp().open();
        mover.grab();
        sleep(300 /* ms */);

        arm.getVertical().moveToPlace(1);
        sleep(300);

        log("Stone dropped & movers in grab position");

        log("Retracting horizontal");
        arm.getHorizontal().moveAllTheWayIn();
    }

    @Override
    protected final void moveToGrabFoundation(RRMecanumDriveBase drive, QuarryState quarryState, final MarkIArm arm, final MarkIIntake intake) {
        Pose2d middleStopPos = releaseFirstStoneMiddleStopPosition(quarryState);
        Vector2d startArmMovementPos = new Vector2d(
            middleStopPos.getX() + inches(12),
            middleStopPos.getY()
        );

        drive.followTrajectorySync(drive.trajectoryBuilder()
                                        .setReversed(true)
                                        .splineTo(releaseFirstStoneMiddleStopPosition(quarryState))
                                        .addMarker(1.5, new Function0<Unit>() {
                                            @Override
                                            public Unit invoke() {
                                                intake.stop();
                                                arm.getClamp().close();
                                                return Unit.INSTANCE;
                                            }
                                        })
                                        // Position stone to be deposited
                                        .addMarker(startArmMovementPos, new Function0<Unit>() {
                                            @Override
                                            public Unit invoke() {
                                                intake.stop();
                                                arm.getClamp().close();
                                                arm.getVertical().moveToPlace(1);
                                                arm.getHorizontal().moveAllTheWayOut();
                                                return Unit.INSTANCE;
                                            }
                                        })
                                        .splineTo(new Pose2d(grabFoundationPosition(), headingTowardsHomeWall()))
                                        .build());
    }

    public static double FOUNDATION_TURN_ALIGN_ANGLE = 17.0;
    public static double FOUNDATION_BACKOUT_INCHES = 32.0;

    protected abstract double foundationAlignHeading();

    @Override
    protected final void moveFoundationToBuildingZoneAndRetractArm(RRMecanumDriveBase drive, final MarkIArm arm) {

        //Partial turn
        turnToHeading(drive, foundationAlignHeading());

        //Move forward a bit
        drive.followTrajectorySync(drive.trajectoryBuilder()
                                    .forward(inches(FOUNDATION_BACKOUT_INCHES)).build());

        // Zero out arm : necessary to perform in auto so we have a "fresh start" for TeleOp
        arm.getVertical().moveToZero();

        //Finish off the turn to get the foundation into position
        turnToHeading(drive, headingTowardsDepotWall());
    }

    public static double SKYBRIDGE_MID_X_IN = 0;
    public static double SKYBRIDGE_MID_Y_IN = 45;

    private Vector2d skybridgeMiddlePosition() {
        return sidedInchesVector(SKYBRIDGE_MID_X_IN, SKYBRIDGE_MID_Y_IN);
    }

    private Vector2d parkPosition() {
        return skybridgeMiddlePosition();
    }

    @Override
    protected final void park(RRMecanumDriveBase drive, final MarkIArm arm) {
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .splineTo(new Pose2d(parkPosition(), headingTowardsDepotWall()))
                .addMarker(0.6, new Function0<Unit>() {
                    @Override
                    public Unit invoke() {
                        log("Retracting vertical");
                        arm.getVertical().moveToZero(); // Completely reset arm for teleop
                        return Unit.INSTANCE;
                    }
                })
                .build());
    }
}
