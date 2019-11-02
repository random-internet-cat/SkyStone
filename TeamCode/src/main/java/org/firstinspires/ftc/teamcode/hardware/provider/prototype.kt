package org.firstinspires.ftc.teamcode.hardware.provider

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.hardware.PrototypeHardware
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.hardware.arm.PrototypeArm
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.MecanumDrive
import org.firstinspires.ftc.teamcode.hardware.drive.tank.TankDrive
import org.firstinspires.ftc.teamcode.hardware.imu.InternalIMU
import org.firstinspires.ftc.teamcode.util.*

private object PrototypeHardwareProvider {
    fun makeIMU(hardwareMap: HardwareMap): InternalIMU {
        val hub = hardwareMap.getHub()
        return InternalIMU.makeOptimized(hub)
    }

    fun makeDrive(hardwareMap: HardwareMap): TankDrive {
        val imu = makeIMU(hardwareMap)
        val frontLeft = hardwareMap.getMotorEx("front_left")
        val frontRight = hardwareMap.getMotorEx("front_right")
        val backLeft = hardwareMap.getMotorEx("back_left")
        val backRight = hardwareMap.getMotorEx("back_right")

        backLeft.direction = DcMotorSimple.Direction.REVERSE
        backRight.direction = DcMotorSimple.Direction.REVERSE

        val drive = TankDrive(imu, frontLeft, frontRight, backLeft, backRight)
        drive.disableEncoders()
        return drive
    }

    fun makeIntake(hardwareMap: HardwareMap): Intake {
        val firstMotor = hardwareMap.getMotor("intake")
        firstMotor.direction = DcMotorSimple.Direction.REVERSE

        val secondMotor = hardwareMap.getMotor("intake2")

        return Intake(firstMotor, secondMotor)
    }

    fun makeArmRotator(hardwareMap: HardwareMap) = PrototypeArm.Rotator(hardwareMap.getMotorEx("arm").also {
        it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        it.mode = DcMotor.RunMode.RUN_USING_ENCODER
        it.setPID(DcMotor.RunMode.RUN_USING_ENCODER, 10.0, 0.0, 10.0)
    })

    fun makeArmWrist(hardwareMap: HardwareMap) = PrototypeArm.Wrist(hardwareMap.getServo("wrist"))
    fun makeArmClamp(hardwareMap: HardwareMap) = PrototypeArm.Clamp(hardwareMap.getServo("clamp"))

    fun makeArm(hardwareMap: HardwareMap) = PrototypeArm(makeArmRotator(hardwareMap), makeArmWrist(hardwareMap), makeArmClamp(hardwareMap))
}

fun makePrototypeHardware(hardwareMap: HardwareMap) = PrototypeHardware(
    PrototypeHardwareProvider.makeArm(hardwareMap),
    PrototypeHardwareProvider.makeIntake(hardwareMap),
    PrototypeHardwareProvider.makeDrive(hardwareMap)
)