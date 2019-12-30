package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.RRMecanumDriveBase;
import org.firstinspires.ftc.teamcode.hardware.MarkIHardware;
import org.firstinspires.ftc.teamcode.hardware.foundation_mover.MarkIFoundationMover;
import org.firstinspires.ftc.teamcode.hardware.provider.MarkIHardwareProvider;

import static org.firstinspires.ftc.teamcode.util.RRUnits.degHeading;
import static org.firstinspires.ftc.teamcode.util.RRUnits.inches;
import static org.firstinspires.ftc.teamcode.util.RRUnits.inchesVector;

public abstract class AutoBase extends LinearOpMode {
    enum QuarryState {
        CLOSE_TO_BRIDGES(inches(-36)),
        MIDDLE(inches(-43)),
        CLOSE_TO_WALL(inches(-50)),
        ;

        private final double xPosition;

        private QuarryState(double xPosition) {
            this.xPosition = xPosition;
        }

        public double xPosition() {
            return xPosition;
        }
    }

    enum SkystoneRelativePos {
        LEFT,
        MIDDLE,
        RIGHT
        ;
    }

    protected final SkystoneRelativePos readQuarryRelative() {
        // TODO: actually figure out where stones are
        return SkystoneRelativePos.RIGHT;
    }

    protected abstract QuarryState mapQuarryState(SkystoneRelativePos relativePos);

    private QuarryState readQuarryState() { return mapQuarryState(readQuarryRelative()); }

    protected final void checkInterrupted() throws InterruptedException {
        if (isStopRequested() || Thread.currentThread().isInterrupted()) throw new InterruptedException();
    }

    protected abstract Pose2d startPosition();

    private void setupDrive(RRMecanumDriveBase drive) {
        drive.setPoseEstimate(startPosition());
    }

    protected abstract double grabStoneYPos();
    protected abstract double grabStoneHeading();

    private void moveToGrabStoneInternal(RRMecanumDriveBase drive, QuarryState quarryState) {
        drive.followTrajectorySync(drive.trajectoryBuilder()
                                        .setReversed(false)
                                        .splineTo(new Pose2d(
                                            quarryState.xPosition(),
                                            grabStoneYPos(),
                                            grabStoneHeading()
                                        )).build());
    }

    private void moveToGrabStone(RRMecanumDriveBase drive) {
        moveToGrabStoneInternal(drive, readQuarryState());
    }

    private void grabFoundation(MarkIFoundationMover foundationMover) {
        foundationMover.grab();
        sleep(1500 /* ms */);
    }

    private void releaseFoundation(MarkIFoundationMover foundationMover) {
        foundationMover.release();
        sleep(1500 /* ms */);
    }

    protected abstract void moveToGrabFoundation(RRMecanumDriveBase drive);
    protected abstract void moveFoundationToQuarry(RRMecanumDriveBase drive);
    protected abstract void park(RRMecanumDriveBase drive);

    @Override
    public final void runOpMode() throws InterruptedException {
        MarkIHardware hardware = MarkIHardwareProvider.makeHardware(hardwareMap);
        RRMecanumDriveBase drive = hardware.getDrive().roadrunner();

        setupDrive(drive);

        waitForStart();

        checkInterrupted();

        moveToGrabStone(drive);

        checkInterrupted();

        moveToGrabFoundation(drive);

        checkInterrupted();

        grabFoundation(hardware.getFoundationMover());

        checkInterrupted();

        moveFoundationToQuarry(drive);

        checkInterrupted();

        releaseFoundation(hardware.getFoundationMover());

        checkInterrupted();

        park(drive);

        checkInterrupted();
    }
}
