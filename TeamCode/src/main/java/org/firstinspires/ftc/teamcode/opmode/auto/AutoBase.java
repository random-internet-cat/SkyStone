package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.RRMecanumDriveBase;
import org.firstinspires.ftc.teamcode.hardware.MarkIHardware;
import org.firstinspires.ftc.teamcode.hardware.provider.MarkIHardwareProvider;

import static org.firstinspires.ftc.teamcode.util.RRUnits.degHeading;
import static org.firstinspires.ftc.teamcode.util.RRUnits.inches;
import static org.firstinspires.ftc.teamcode.util.RRUnits.inchesVector;
import static org.firstinspires.ftc.teamcode.util.RRUnits.zeroDistance;
import static org.firstinspires.ftc.teamcode.util.RRUnits.zeroHeading;

@Autonomous
public class AutoBase extends LinearOpMode {
    enum QuarryState {
        CLOSE_TO_BRIDGES(inchesVector(-36, 33)),
        MIDDLE(inchesVector(-43, 33)),
        CLOSE_TO_WALL(inchesVector(-50, 33)),
        ;

        private final Vector2d grabPosition;

        private QuarryState(Vector2d grabPosition) {
            this.grabPosition = grabPosition;
        }

        public Vector2d grabPosition() {
            return grabPosition;
        }
    }

    private QuarryState readQuarry() {
        // TODO: actually figure out where stones are
        return QuarryState.CLOSE_TO_BRIDGES;
    }

    private void checkInterrupted() throws InterruptedException {
        if (isStopRequested() || Thread.currentThread().isInterrupted()) throw new InterruptedException();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        MarkIHardware hardware = MarkIHardwareProvider.makeHardware(hardwareMap);
        RRMecanumDriveBase drive = hardware.getDrive().roadrunner();

        drive.setPoseEstimate(new Pose2d(inchesVector(-32.0, 60.0), degHeading(90)));

        waitForStart();

        checkInterrupted();

        QuarryState quarryState = readQuarry();
        // TODO: can this be null? if it can, implement error handling
        if (quarryState == null) quarryState = QuarryState.CLOSE_TO_BRIDGES;

        // Drive to first skystone
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .setReversed(true)
                .splineTo(new Pose2d(
                        quarryState.grabPosition(),
                        degHeading(90)
                )).build());

        checkInterrupted();

        // Clamp skystone
        hardware.getFoundationMover().grab();
        //sleep(1000);

        checkInterrupted();

        // Move back
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .setReversed(false)
                .forward(inches(20))
                .build());

        checkInterrupted();

        // Spline to foundation
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .setReversed(true)
                .splineTo(new Pose2d(inchesVector(12, 36), degHeading(180)))
                .splineTo(new Pose2d(inchesVector(48, 36), degHeading(180)))
                .build());

        checkInterrupted();

        // Un clamp skystone
        hardware.getFoundationMover().release();
        sleep(1000);

        checkInterrupted();

        // Turn to face foundation
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .setReversed(true)
                .splineTo(new Pose2d(inchesVector(48, 32), degHeading(90)))
                .build());

        checkInterrupted();

        // Clamp foundation
        hardware.getFoundationMover().grab();
        sleep(1500);

        checkInterrupted();

        // Spline to make foundation horizontal
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .setReversed(false)
                .splineTo(new Pose2d(inchesVector(30, 42), degHeading(180)))
                .build());

        checkInterrupted();

        // Drive forward to push foundation into building site
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .forward(-inches(16))
                .build());

        checkInterrupted();

        // Un clamp foundation
        hardware.getFoundationMover().release();

        checkInterrupted();

        // Park spline
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .setReversed(false)
                .splineTo(new Pose2d(inchesVector(0, 36), degHeading(180)))
                .build());

        checkInterrupted();
    }
}
