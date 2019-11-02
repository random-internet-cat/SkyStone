package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.hardware.arm.PrototypeArm
import org.firstinspires.ftc.teamcode.hardware.drive.tank.TankDrive
import org.firstinspires.ftc.teamcode.intake.Intake

typealias PrototypeIntake = Intake
typealias PrototypeDrive = TankDrive

class PrototypeHardware(private val arm: PrototypeArm, private val intake: PrototypeIntake, private val drive: PrototypeDrive) {
    fun arm() = arm
    fun intake() = intake
    fun drive() = drive

    fun update() {
        arm().update()
        intake().update()
        drive().update()
    }
}