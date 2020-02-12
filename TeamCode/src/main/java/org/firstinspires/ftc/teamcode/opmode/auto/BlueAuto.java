package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.MarkIHardware;
import org.firstinspires.ftc.teamcode.hardware.auto_claw.MarkIAutoClaws;
import org.firstinspires.ftc.teamcode.hardware.foundation_mover.MarkIFoundationMover;

import java.util.Objects;

import static org.firstinspires.ftc.teamcode.util.RRUnits.degHeading;

@Autonomous
public class BlueAuto extends SidedAutoBase {
    public BlueAuto() {
        super(SideColor.BLUE);
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
    protected void prepareToGrabStone(MarkIHardware hardware) {}

    @Override
    protected double grabStoneCurveHeading() {
        return degHeading(240);
    }

    @Override
    protected double grabStoneHeading() {
        return degHeading(240);
    }

    @Override
    protected double headingTowardsFoundationWall() {
        return degHeading(0);
    }

    @Override
    protected double headingTowardsHomeWall() {
        return degHeading(90);
    }
}
