package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.hardware.arm.PrototypeArm
import org.firstinspires.ftc.teamcode.hardware.drive.tank.TankDrive
import org.firstinspires.ftc.teamcode.hardware.intake.PrototypeIntake

typealias PrototypeDrive = TankDrive

data class PrototypeHardware(val arm: PrototypeArm, val intake: PrototypeIntake, val drive: PrototypeDrive) {
    fun update() {
        arm.update()
        intake.update()
        drive.update()
    }
}