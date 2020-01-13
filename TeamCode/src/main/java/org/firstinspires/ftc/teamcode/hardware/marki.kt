package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.hardware.arm.MarkIArm
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.MecanumDrive
import org.firstinspires.ftc.teamcode.hardware.foundation_mover.MarkIFoundationMover
import org.firstinspires.ftc.teamcode.hardware.intake.MarkIIntake
import org.firstinspires.ftc.teamcode.hardware.intake_flippers.MarkIIntakeFlippers
import org.firstinspires.ftc.teamcode.hardware.rear_claw.MarkIRearClaws

typealias MarkIDrivetrain = MecanumDrive

data class MarkIHardware(val drive: MarkIDrivetrain, val arm: MarkIArm, val intake: MarkIIntake, val intakeFlippers: MarkIIntakeFlippers, val foundationMover: MarkIFoundationMover, val rearClaws: MarkIRearClaws) {
    fun update() {
        drive.update()
        arm.update()
        intake.update()
        intakeFlippers.update()
        foundationMover.update()
        rearClaws.update()
    }
}