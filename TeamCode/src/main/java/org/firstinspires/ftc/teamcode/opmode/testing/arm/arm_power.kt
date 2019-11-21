package org.firstinspires.ftc.teamcode.opmode.testing.arm

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.util.*

@Autonomous
@Config
class ArmPowerTest : LinearOpMode() {
    companion object {
        @JvmField
        public var POWER: Double = 0.5
    }

    override fun runOpMode() {
        val dashboard = FtcDashboard.getInstance()
        val motor = TypedMotor(hardwareMap.getMotor("arm"), externalGearing = 1.0)

        waitForStart()

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)

        while (opModeIsActive()) {
            if (!gamepad1.a) {
                motor.setPower(POWER)
            } else {
                motor.setPower(0)
            }
        }
    }
}