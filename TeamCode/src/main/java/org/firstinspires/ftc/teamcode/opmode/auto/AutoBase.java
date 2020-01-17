package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.RRMecanumDriveBase;
import org.firstinspires.ftc.teamcode.hardware.MarkIHardware;
import org.firstinspires.ftc.teamcode.hardware.foundation_mover.MarkIFoundationMover;
import org.firstinspires.ftc.teamcode.hardware.intake_flippers.MarkIIntakeFlippers;
import org.firstinspires.ftc.teamcode.hardware.provider.MarkIHardwareProvider;

import static org.firstinspires.ftc.teamcode.util.RRUnits.inches;

@Config
public abstract class AutoBase extends LinearOpMode {
    public static double _STONE_CLOSE_TO_BRIDGES_X_IN = -33;
    public static double _STONE_MIDDLE_X_IN = -40;
    public static double _STONE_CLOSE_TO_WALL_X_IN = -47;

    public enum QuarryState {
        CLOSE_TO_BRIDGES {
            @Override
            public double xPosition() {
                return inches(_STONE_CLOSE_TO_BRIDGES_X_IN);
            }
        },
        MIDDLE {
            @Override
            public double xPosition() {
                return inches(_STONE_MIDDLE_X_IN);
            }
        },
        CLOSE_TO_WALL {
            @Override
            public double xPosition() {
                return inches(_STONE_CLOSE_TO_WALL_X_IN);
            }
        },
        ;

        public abstract double xPosition();
    }

    enum SkystoneRelativePos {
        LEFT,
        MIDDLE,
        RIGHT
        ;
    }

    protected abstract QuarryState readQuarryState();

    protected final void checkInterrupted() throws InterruptedException {
        if (isStopRequested() || Thread.currentThread().isInterrupted()) throw new InterruptedException();
    }

    protected abstract Pose2d startPosition();

    private void setupDrive(RRMecanumDriveBase drive) {
        drive.setPoseEstimate(startPosition());
    }

    protected abstract void turnTowardsWall(RRMecanumDriveBase drive);

    private void releaseIntake(MarkIIntakeFlippers flippers) {
        flippers.release();
        sleep(500);
    }

    protected abstract void prepareToGrabStone(MarkIHardware hardware, QuarryState quarryState);
    protected abstract void moveToGrabStone(RRMecanumDriveBase drive, QuarryState quarryState);
    protected abstract void grabStone(MarkIHardware hardware, QuarryState quarryState);

    private void grabFoundation(MarkIFoundationMover foundationMover) {
        foundationMover.grabBoth();
        sleep(1500 /* ms */);
    }

    private void releaseFoundation(MarkIFoundationMover foundationMover) {
        foundationMover.releaseBoth();
        sleep(1500 /* ms */);
    }

    protected abstract void prepareToGrabFoundation(MarkIHardware hardware);
    protected abstract void moveToGrabFoundation(RRMecanumDriveBase drive);
    protected abstract void releaseStone(MarkIHardware hardware, QuarryState quarryState);
    protected abstract void moveFoundationToBuildingZone(RRMecanumDriveBase drive);
    protected abstract void park(RRMecanumDriveBase drive);

    protected final void log(String message) {
        telemetry.log().add(message);
        telemetry.update();
    }

    @Override
    public final void runOpMode() throws InterruptedException {
        log("Initializing...");

        MarkIHardware hardware = MarkIHardwareProvider.makeHardware(hardwareMap);
        RRMecanumDriveBase drive = hardware.getDrive().roadrunner();

        QuarryState quarryState = readQuarryState();

        setupDrive(drive);

        log("Initialized. Waiting for start.");

        waitForStart();

        log("Starting!");

        checkInterrupted();

        log("Turning to face wall");
        turnTowardsWall(drive);
        log("Turned to face wall");

        checkInterrupted();

        log("Preparing to grab stone");
        prepareToGrabStone(hardware, quarryState);
        log("Prepared to grab stone");

        checkInterrupted();

        log("Moving to grab stone");
        moveToGrabStone(drive, quarryState);
        log("Moved to grab stone");

        checkInterrupted();

        log("Grabbing stone");
        grabStone(hardware, quarryState);
        log("Grabbed stone");

        checkInterrupted();

        log("Preparing to grab foundation");
        prepareToGrabFoundation(hardware);
        log("Prepared to grab foundation");

        log("Moving to grab foundation");
        moveToGrabFoundation(drive);
        log("Moved to grab foundation");

        checkInterrupted();

        log("Releasing stone");
        releaseStone(hardware, quarryState);
        log("Released stone");

        checkInterrupted();

        log("Grabbing foundation");
        grabFoundation(hardware.getFoundationMover());
        log("Grabbed foundation");

        checkInterrupted();

        log("Moving foundation to depot");
        moveFoundationToBuildingZone(drive);
        log("Moved foundation to depot");

        checkInterrupted();

        log("Releasing foundation");
        releaseFoundation(hardware.getFoundationMover());
        log("Released foundation");

        checkInterrupted();

        log("Parking");
        park(drive);
        log("Parked");

        checkInterrupted();
    }
}
