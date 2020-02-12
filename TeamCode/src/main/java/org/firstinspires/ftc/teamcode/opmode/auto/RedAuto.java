package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.MarkIHardware;
import org.firstinspires.ftc.teamcode.hardware.auto_claw.MarkIAutoClaws;

import java.util.Objects;

import static org.firstinspires.ftc.teamcode.util.RRUnits.degHeading;

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
    protected double grabStoneCurveHeading() {
        return degHeading(135);
    }

    @Override
    protected double grabStoneHeading() {
        return degHeading(150);
    }

    @Override
    protected double headingTowardsFoundationWall() {
        return degHeading(0);
    }

    @Override
    protected double headingTowardsHomeWall() {
        return degHeading(270);
    }
}
