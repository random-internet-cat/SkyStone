package org.firstinspires.ftc.teamcode.opmode.testing.arm

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.hardware.arm.PrototypeArm
import org.firstinspires.ftc.teamcode.util.getMotor
import org.firstinspires.ftc.teamcode.util.units.DegreesPoint

class ArmAngleReader : LinearOpMode() {
    override fun runOpMode() {
        val arm = PrototypeArm.Rotator(hardwareMap.getMotor("arm")).roadrunner()

        while (opModeIsActive()) {
            telemetry.addData("Arm angle", DegreesPoint(arm.currentAngle()).raw)
            telemetry.update()
        }
    }
}