package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.mecanum.RRMecanumDriveBase;
import org.firstinspires.ftc.teamcode.hardware.MarkIHardware;
import org.firstinspires.ftc.teamcode.hardware.imu.InternalIMU;
import org.firstinspires.ftc.teamcode.util.Hardware_mapKt;

import java.io.File;
import java.util.Objects;

import static org.firstinspires.ftc.teamcode.util.RRUnits.degHeading;
import static org.firstinspires.ftc.teamcode.util.RRUnits.inches;

@Autonomous
public class RedAuto extends SidedAutoBase {
    public RedAuto() {
        super(SideColor.RED);
    }

    @Override
    protected QuarryState mapQuarryState(SkystoneRelativePos relativePos) {
        Objects.requireNonNull(relativePos, "relativePos");

        switch (relativePos) {
            case RIGHT: return QuarryState.CLOSE_TO_BRIDGES;
            case MIDDLE: return QuarryState.MIDDLE;
            case LEFT: return QuarryState.CLOSE_TO_WALL;
        }

        throw new UnsupportedOperationException();
    }

    @Override
    protected void prepareToGrabStone(MarkIHardware hardware) {}

    @Override
    protected double grabFirstStoneHeading() {
        return degHeading(120);
    }

    @Override
    protected double grabSecondStoneHeading(QuarryState quarryState) {
        if (quarryState == QuarryState.CLOSE_TO_WALL) {
            return degHeading(130); // TODO
        } else {
            return degHeading(120);
        }
    }

    @Override
    protected double headingTowardsFoundationWall() {
        return degHeading(0);
    }

    @Override
    protected double headingTowardsHomeWall() {
        return degHeading(270);
    }

    @Override
    protected double foundationAlignHeading() {
        return headingTowardsHomeWall() - degHeading(FOUNDATION_TURN_ALIGN_ANGLE);
    }

    public static double GRAB_FOUNDATION_X_IN = 45.5;
    public static double GRAB_FOUNDATION_Y_IN = 26;

    @Override
    protected Vector2d grabFoundationPosition() {
        return sidedInchesVector(GRAB_FOUNDATION_X_IN, GRAB_FOUNDATION_Y_IN);
    }

    public static double GRAB_SECOND_STONE_Y_POS_IN = 25;

    @Override
    protected double grabSecondStoneYPos() {
        return ySidedInches(GRAB_SECOND_STONE_Y_POS_IN);
    }

    public static double DEPOSIT_SECOND_STONE_MIDPOINT_X_IN = 0;
    public static double DEPOSIT_SECOND_STONE_MIDPOINT_Y_IN = 43;

    @Override
    protected Pose2d depositSecondStoneMidpointPosition() {
        return new Pose2d(
            sidedInchesVector(
                DEPOSIT_SECOND_STONE_MIDPOINT_X_IN,
                DEPOSIT_SECOND_STONE_MIDPOINT_Y_IN
            ),
            headingTowardsDepotWall()
        );
    }

    public static double FIRST_STONE_CLOSE_TO_BRIDGES_X_IN = -22;
    public static double FIRST_STONE_MIDDLE_X_IN = -30;
    public static double FIRST_STONE_CLOSE_TO_WALL_X_IN = -38;

    @Override
    protected double firstStoneGrabXPosition(QuarryState quarryState) {
        switch (quarryState) {
            case CLOSE_TO_BRIDGES: return inches(FIRST_STONE_CLOSE_TO_BRIDGES_X_IN);
            case MIDDLE: return inches(FIRST_STONE_MIDDLE_X_IN);
            case CLOSE_TO_WALL: return inches(FIRST_STONE_CLOSE_TO_WALL_X_IN);
        }

        throw new AssertionError();
    }

    public static double SECOND_STONE_CLOSE_TO_BRIDGES_X_IN = -48;
    public static double SECOND_STONE_MIDDLE_X_IN = -56;
    public static double SECOND_STONE_CLOSE_TO_WALL_X_IN = -64;

    @Override
    protected double secondStoneGrabXPosition(QuarryState quarryState) {
        switch (quarryState) {
            case CLOSE_TO_BRIDGES: return inches(SECOND_STONE_CLOSE_TO_BRIDGES_X_IN);
            case MIDDLE: return inches(SECOND_STONE_MIDDLE_X_IN);
            case CLOSE_TO_WALL: return inches(SECOND_STONE_CLOSE_TO_WALL_X_IN);
        }

        throw new AssertionError();
    }
}
