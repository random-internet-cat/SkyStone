package org.firstinspires.ftc.teamcode.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.hardware.provider.makeMarkIHardware

@Autonomous
class AutoClawGrab : LinearOpMode() {
    override fun runOpMode() {
        val hardware = makeMarkIHardware(hardwareMap)

        waitForStart()

        while (opModeIsActive()) {
            hardware.autoClaws.clampBoth()
        }
    }
}