package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.hardware.arm.MarkIArm
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.MecanumDrive
import org.firstinspires.ftc.teamcode.hardware.foundation_mover.MarkIFoundationMover
import org.firstinspires.ftc.teamcode.hardware.intake.MarkIIntake
import org.firstinspires.ftc.teamcode.hardware.auto_claw.MarkIAutoClaws

typealias MarkIDrivetrain = MecanumDrive

data class MarkIHardware(val drive: MarkIDrivetrain, val arm: MarkIArm, val intake: MarkIIntake, val foundationMover: MarkIFoundationMover, val autoClaws: MarkIAutoClaws) {
    fun update(updateDashboard: Boolean = true) {
        drive.update(updateDashboard = updateDashboard)
        arm.update()
        intake.update()
        foundationMover.update()
        autoClaws.update()
    }
}