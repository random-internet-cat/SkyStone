package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.rear_claw.MarkIRearClaws;

import java.util.Objects;

import static org.firstinspires.ftc.teamcode.util.RRUnits.degHeading;

@Autonomous
public class RedAuto extends SidedAutoBase {
    public RedAuto() {
        super(SideYSign.POSITIVE);
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
    protected void clampRearClawsForStone(MarkIRearClaws claws) {
        claws.clampLeft();
    }

    @Override
    protected void releaseRearClawsForStone(MarkIRearClaws claws) {
        claws.releaseLeft();
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
