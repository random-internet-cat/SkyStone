package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.mecanum.RRMecanumDriveBase;

import static org.firstinspires.ftc.teamcode.util.RRUnits.inches;

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

    SidedAutoBase(SideYSign ySign) {
        this.ySign = ySign;
    }

    private double sideY(double raw) {
        return ySign.scale(raw);
    }

    public static double GRAB_STONE_Y_POS_IN = 33;

    private double grabStoneYPos() {
        return sideY(inches(GRAB_STONE_Y_POS_IN));
    }

    protected abstract double grabStoneHeading();

    @Override
    protected final void moveToGrabStoneInternal(RRMecanumDriveBase drive, QuarryState quarryState) {
        drive.followTrajectorySync(drive.trajectoryBuilder()
                                        .setReversed(false)
                                        .splineTo(new Pose2d(
                                            quarryState.xPosition(),
                                            grabStoneYPos(),
                                            grabStoneHeading()
                                        )).build());
    }
}
