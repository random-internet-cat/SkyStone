package org.firstinspires.ftc.teamcode.hardware.provider

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.hardware.*
import org.firstinspires.ftc.teamcode.hardware.arm.MarkIArm
import org.firstinspires.ftc.teamcode.hardware.drive.*
import org.firstinspires.ftc.teamcode.hardware.drive.constants.MarkIDriveConstants
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.*
import org.firstinspires.ftc.teamcode.hardware.foundation_mover.BaseFoundationMover
import org.firstinspires.ftc.teamcode.hardware.foundation_mover.MarkIFoundationMover
import org.firstinspires.ftc.teamcode.hardware.intake.MarkIIntake
import org.firstinspires.ftc.teamcode.util.*
import org.firstinspires.ftc.teamcode.util.roadrunner.PIDCoefficients

private object MarkIHardwareProvider {
    fun makeDrive(hardwareMap: HardwareMap): MarkIDrivetrain {
        val imu = hardwareMap.getIMU()

        val frontLeft = hardwareMap.getMotorEx("front_left")
        val frontRight = hardwareMap.getMotorEx("front_right")
        val backLeft = hardwareMap.getMotorEx("back_left")
        val backRight = hardwareMap.getMotorEx("back_right")

        frontLeft.setReversed()
        backLeft.setReversed()

        val externalGearing = 1.0

        val drivetrain = MecanumDrivetrain(
            frontLeft = TypedMotorEx(frontLeft, externalGearing = externalGearing),
            frontRight = TypedMotorEx(frontRight, externalGearing = externalGearing),
            backLeft = TypedMotorEx(backLeft, externalGearing = externalGearing),
            backRight = TypedMotorEx(backRight, externalGearing = externalGearing)
        )

        val drive = MecanumDrive(MecanumUseHeadingProvider(IMUHeadingProvider(imu)), MarkIDriveConstants, drivetrain)
        drive.enableEncoders()
        drive.brakeOnZeroPower()
        drive.setPID(DcMotor.RunMode.RUN_USING_ENCODER, PIDCoefficients(33.0, 0.0, 8.0))
        return drive
    }

    fun makeArmRotator(hardwareMap: HardwareMap) = MarkIArm.Rotator(TypedMotorEx(hardwareMap.getMotorEx("arm"), externalGearing = 0.5).also {
        it.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        it.motor.targetPosition = 0
        it.motor.resetEncoder()
        it.motor.setReversed()
        it.brakeOnZeroPower()
        it.motor.setPID(DcMotor.RunMode.RUN_USING_ENCODER, 10.0, 0.0, 10.0)
    })

    fun makeArmWrist(hardwareMap: HardwareMap) = MarkIArm.Wrist(hardwareMap.getServo("wrist"))
    fun makeArmClamp(hardwareMap: HardwareMap) = MarkIArm.Clamp(hardwareMap.getServo("clamp"))

    fun makeArm(hardwareMap: HardwareMap): MarkIArm = MarkIArm(makeArmRotator(hardwareMap), makeArmWrist(hardwareMap), makeArmClamp(hardwareMap))

    fun makeIntake(hardwareMap: HardwareMap): MarkIIntake {
        val firstMotor = hardwareMap.getMotor("intake")
        val secondMotor = hardwareMap.getMotor("intake2")

        firstMotor.setReversed()

        return MarkIIntake(firstMotor, secondMotor)
    }

    fun makeFoundationMover(hardwareMap: HardwareMap): MarkIFoundationMover {
        val firstServo = hardwareMap.getServo("foundation_mover_1")
        val secondServo = hardwareMap.getServo("foundation_mover_2")
        secondServo.direction = Servo.Direction.REVERSE

        return MarkIFoundationMover(firstServo, secondServo)
    }
}

fun makeMarkIArm(hardwareMap: HardwareMap) = MarkIHardwareProvider.makeArm(hardwareMap)
fun makeMarkIIntake(hardwareMap: HardwareMap) = MarkIHardwareProvider.makeIntake(hardwareMap)
fun makeMarkIDrive(hardwareMap: HardwareMap) = MarkIHardwareProvider.makeDrive(hardwareMap)
fun makeMarkIHardware(hardwareMap: HardwareMap) = MarkIHardware(MarkIHardwareProvider.makeDrive(hardwareMap), MarkIHardwareProvider.makeArm(hardwareMap), MarkIHardwareProvider.makeIntake(hardwareMap), MarkIHardwareProvider.makeFoundationMover(hardwareMap))