package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.RRMecanumDriveBase;
import org.firstinspires.ftc.teamcode.hardware.MarkIHardware;
import org.firstinspires.ftc.teamcode.hardware.provider.MarkIHardwareProvider;

import static org.firstinspires.ftc.teamcode.util.RRUnits.inchesVector;
import static org.firstinspires.ftc.teamcode.util.RRUnits.zeroDistance;
import static org.firstinspires.ftc.teamcode.util.RRUnits.zeroHeading;

public abstract class AutoBase extends LinearOpMode {
    enum QuarryState {
        CLOSE_TO_BRIDGES,
        MIDDLE,
        CLOSE_TO_WALL
    }

    private QuarryState readQuarry() {
        // TODO: actually figure out where stones are
        return QuarryState.CLOSE_TO_BRIDGES;
    }

    @Override
    public void runOpMode() {
        MarkIHardware hardware = MarkIHardwareProvider.makeHardware(hardwareMap);
        RRMecanumDriveBase drive = hardware.getDrive().roadrunner();

        waitForStart();

        drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(
            inchesVector(30, 30),
            zeroHeading()
        )).build());

        sleep(2000 /* ms */);

        drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(
            zeroDistance(),
            zeroDistance(),
            zeroHeading()
        )).build());
    }
}
