package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.hardware.arm.PrototypeArm
import org.firstinspires.ftc.teamcode.hardware.drive.tank.TankDrive
import org.firstinspires.ftc.teamcode.intake.Intake

class PrototypeHardware(private val arm: PrototypeArm, private val intake: Intake, private val drive: TankDrive) {
    fun arm() = arm
    fun intake() = intake
    fun drive() = drive

    fun update() {
        arm().update()
        intake().update()
        drive().update()
    }
}