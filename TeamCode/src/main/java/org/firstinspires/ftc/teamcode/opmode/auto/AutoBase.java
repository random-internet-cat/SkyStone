package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.RRMecanumDriveBase;
import org.firstinspires.ftc.teamcode.hardware.MarkIHardware;
import org.firstinspires.ftc.teamcode.hardware.foundation_mover.MarkIFoundationMover;
import org.firstinspires.ftc.teamcode.hardware.provider.MarkIHardwareProvider;

import static org.firstinspires.ftc.teamcode.util.RRUnits.inches;

@Config
public abstract class AutoBase extends LinearOpMode {
    public static double _STONE_CLOSE_TO_BRIDGES_X_IN = -33;
    public static double _STONE_MIDDLE_X_IN = -42;
    public static double _STONE_CLOSE_TO_WALL_X_IN = -51;

    public static double _CORRESPONDING_STONE_DISTANCE_IN = 30;

    public enum QuarryState {
        CLOSE_TO_BRIDGES {
            @Override
            public double firstXPosition() {
                return inches(_STONE_CLOSE_TO_BRIDGES_X_IN);
            }
        },
        MIDDLE {
            @Override
            public double firstXPosition() {
                return inches(_STONE_MIDDLE_X_IN);
            }
        },
        CLOSE_TO_WALL {
            @Override
            public double firstXPosition() {
                return inches(_STONE_CLOSE_TO_WALL_X_IN);
            }
        },
        ;

        public abstract double firstXPosition();
        public double secondXPosition() { return firstXPosition() + inches(_CORRESPONDING_STONE_DISTANCE_IN); }
    }

    public enum SkystoneRelativePos {
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

    private void setupHardware(MarkIHardware hardware) {
        hardware.getAutoClaws().releaseBoth();
        hardware.getFoundationMover().moveBothToOutOfTheWay();
    }

    protected abstract void turnTowardsWall(RRMecanumDriveBase drive);

    private void grabFoundation(MarkIFoundationMover foundationMover) {
        foundationMover.grabBoth();
        sleep(1500 /* ms */);
    }

    private void releaseFoundation(MarkIFoundationMover foundationMover) {
        foundationMover.moveBothToOutOfTheWay();
        sleep(1500 /* ms */);
    }

    protected abstract void prepareToGrabFoundation(MarkIHardware hardware);
    protected abstract void moveToGrabFoundation(RRMecanumDriveBase drive);
    protected abstract void moveFoundationToBuildingZone(RRMecanumDriveBase drive);
    protected abstract void park(RRMecanumDriveBase drive);

    protected final void log(String message) {
        telemetry.log().add(message);
        telemetry.update();
    }

    protected abstract void handleFirstStone(MarkIHardware hardware, RRMecanumDriveBase drive, QuarryState quarryState) throws InterruptedException;
    protected abstract void handleSecondStone(MarkIHardware hardware, RRMecanumDriveBase drive, QuarryState quarryState) throws InterruptedException;

    private void handleFoundation(MarkIHardware hardware, RRMecanumDriveBase drive) throws InterruptedException {
        log("Preparing to grab foundation");
        prepareToGrabFoundation(hardware);
        log("Prepared to grab foundation");

        checkInterrupted();

        log("Moving to grab foundation");
        moveToGrabFoundation(drive);
        log("Moved to grab foundation");

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
    }

    @Override
    public final void runOpMode() throws InterruptedException {
        log("Initializing...");

        MarkIHardware hardware = MarkIHardwareProvider.makeHardware(hardwareMap);
        RRMecanumDriveBase drive = hardware.getDrive().roadrunner();

        setupDrive(drive);
        setupHardware(hardware);

        log("Initialized. Waiting for start.");

        waitForStart();

        checkInterrupted();

        log("Starting!");

        log("Reading quarry state");
        QuarryState quarryState = readQuarryState();
        log("Read quarry state, got: " + quarryState);

        checkInterrupted();

        handleFirstStone(hardware, drive, quarryState);

        //checkInterrupted();
        //
        //handleFoundation(hardware, drive);
        //
        //checkInterrupted();
        //
        //handleSecondStone(hardware, drive, quarryState);
        //
        //log("Parking");
        //park(drive);
        //log("Parked");
        //
        //checkInterrupted();
    }
}
