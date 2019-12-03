package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.hardware.arm.MarkIArm
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.MecanumDrive
import org.firstinspires.ftc.teamcode.hardware.foundation_mover.MarkIFoundationMover
import org.firstinspires.ftc.teamcode.hardware.intake.MarkIIntake

typealias MarkIDrivetrain = MecanumDrive

data class MarkIHardware(val drive: MarkIDrivetrain, val arm: MarkIArm, val intake: MarkIIntake, val foundationMover: MarkIFoundationMover) {
    fun update() {
        drive.update()
        arm.update()
        intake.update()
        foundationMover.update()
    }
}