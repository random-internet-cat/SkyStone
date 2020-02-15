package org.firstinspires.ftc.teamcode.opmode.auto;

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

    @Override
    protected double foundationAlignHeading() {
        return headingTowardsHomeWall() - degHeading(FOUNDATION_TURN_ALIGN_ANGLE);
    }
}
