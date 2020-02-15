package org.firstinspires.ftc.teamcode.opmode.auto;

import android.media.MediaPlayer;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.drive.mecanum.RRMecanumDriveBase;
import org.firstinspires.ftc.teamcode.hardware.MarkIHardware;
import org.firstinspires.ftc.teamcode.hardware.arm.MarkIArm;
import org.firstinspires.ftc.teamcode.hardware.foundation_mover.MarkIFoundationMover;
import org.firstinspires.ftc.teamcode.hardware.imu.InternalIMU;
import org.firstinspires.ftc.teamcode.hardware.intake.MarkIIntake;
import org.firstinspires.ftc.teamcode.hardware.provider.MarkIHardwareProvider;
import org.firstinspires.ftc.teamcode.util.Hardware_mapKt;

import java.io.File;

import static org.firstinspires.ftc.teamcode.util.RRUnits.inches;

@Config
public abstract class AutoBase extends LinearOpMode {
    public static double _STONE_CLOSE_TO_BRIDGES_X_IN = -21;
    public static double _STONE_MIDDLE_X_IN = -28;
    public static double _STONE_CLOSE_TO_WALL_X_IN = -47;

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

    protected abstract QuarryState readQuarryState(SkystoneDetector detector);
    protected abstract void saveGyroDataSided(RRMecanumDriveBase drive);

    private final void resetIMU() {
        InternalIMU imu = Hardware_mapKt.getIMU(hardwareMap, "imu");
        imu.reset();
    }

    protected final void checkInterrupted() throws InterruptedException {
        if (isStopRequested() || Thread.currentThread().isInterrupted()) throw new InterruptedException();
    }

    protected abstract Pose2d startPosition();

    private void setupDrive(RRMecanumDriveBase drive) {
        drive.setPoseEstimate(startPosition());
    }

    private void setupHardware(MarkIHardware hardware) {
        hardware.getFoundationMover().moveBothToOutOfTheWay();
        hardware.getArm().getVertical().moveToCollect();
    }

    protected abstract void turnTowardsWall(RRMecanumDriveBase drive);

    private void grabFoundation(MarkIFoundationMover foundationMover) {
        foundationMover.grabBoth();
        sleep(500 /* ms */);
    }

    private void releaseFoundation(MarkIFoundationMover foundationMover) {
        foundationMover.moveBothToOutOfTheWay();
        sleep(500 /* ms */);
    }

    protected abstract void grabFoundation(MarkIHardware hardware);
    protected abstract void moveToGrabFoundation(RRMecanumDriveBase drive, QuarryState quarryState, final MarkIArm arm, final MarkIIntake intake);
    protected abstract void moveFoundationToBuildingZoneAndRetractArm(RRMecanumDriveBase drive, MarkIArm arm);
    protected abstract void park(RRMecanumDriveBase drive);

    protected final void log(String message) {
        telemetry.log().add(message);
        telemetry.update();
    }

    protected abstract void handleFirstStone(MarkIHardware hardware, QuarryState quarryState) throws InterruptedException;
    protected abstract void handleSecondStone(MarkIHardware hardware, QuarryState quarryState) throws InterruptedException;

    private void handleFoundation(MarkIHardware hardware, QuarryState quarryState) throws InterruptedException {
        RRMecanumDriveBase drive = hardware.getDrive().roadrunner();

        log("Moving to grab foundation");
        moveToGrabFoundation(drive, quarryState, hardware.getArm(), hardware.getIntake());
        log("Moved to grab foundation");

        checkInterrupted();

        log("Grab foundation and outtake stone");
        grabFoundation(hardware);
        log("Grabbed foundation and outtook stone");

        checkInterrupted();

        log("Moving foundation to depot");
        moveFoundationToBuildingZoneAndRetractArm(drive, hardware.getArm());
        log("Moved foundation to depot");

        checkInterrupted();

        log("Releasing foundation");
        releaseFoundation(hardware.getFoundationMover());
        log("Released foundation");
    }

    @Override
    public final void runOpMode() throws InterruptedException {
        MediaPlayer moskauPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.moskau);

        log("Initializing...");

        MarkIHardware hardware = MarkIHardwareProvider.makeHardware(hardwareMap);
        RRMecanumDriveBase drive = hardware.getDrive().roadrunner();
        SkystoneDetector detector = new SkystoneDetector(hardwareMap);

        setupDrive(drive);
        setupHardware(hardware);

        log("Initialized. Waiting for start.");

        waitForStart();

        moskauPlayer.start();

        resetIMU();

        checkInterrupted();

        log("Starting!");

        log("Reading quarry state");
        QuarryState quarryState = readQuarryState(detector);
        log("Read quarry state, got: " + quarryState);

        checkInterrupted();

        handleFirstStone(hardware, quarryState);

        checkInterrupted();

        handleFoundation(hardware, quarryState);

        checkInterrupted();
        //
        //handleSecondStone(hardware, drive, quarryState);
        //
        log("Parking");
        park(drive);
        log("Parked");

        checkInterrupted();

        moskauPlayer.stop();
    }
}
