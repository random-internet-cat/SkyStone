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
import org.firstinspires.ftc.teamcode.hardware.intake_flippers.MarkIIntakeFlippers
import org.firstinspires.ftc.teamcode.util.*
import org.firstinspires.ftc.teamcode.util.roadrunner.PIDCoefficients

object MarkIHardwareProvider {
    @JvmStatic
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

        val drive = MecanumDrive(MecanumUseHeadingProvider(InvertedHeadingProvider(IMUHeadingProvider(imu))), MarkIDriveConstants, drivetrain)
        drive.enableEncoders()
        drive.brakeOnZeroPower()
        drive.roadrunner().setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDCoefficients(33.0, 0.0, 8.0))

        return drive
    }

    @JvmStatic
    fun makeArmRotator(hardwareMap: HardwareMap) = MarkIArm.Rotator(TypedMotorEx(hardwareMap.getMotorEx("arm"), externalGearing = 0.5).also {
        it.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        it.motor.targetPosition = 0
        it.motor.resetEncoder()
        it.motor.setReversed()
        it.brakeOnZeroPower()
        it.motor.setPID(DcMotor.RunMode.RUN_USING_ENCODER, 10.0, 0.0, 10.0)
    })

    @JvmStatic
    fun makeArmWrist(hardwareMap: HardwareMap) = MarkIArm.Wrist(hardwareMap.getServo("wrist"))

    @JvmStatic
    fun makeArmClamp(hardwareMap: HardwareMap) = MarkIArm.Clamp(hardwareMap.getServo("clamp"))

    @JvmStatic
    fun makeArm(hardwareMap: HardwareMap): MarkIArm = MarkIArm(makeArmRotator(hardwareMap), makeArmWrist(hardwareMap), makeArmClamp(hardwareMap))

    @JvmStatic
    fun makeIntake(hardwareMap: HardwareMap): MarkIIntake {
        val firstMotor = hardwareMap.getMotor("intake")
        val secondMotor = hardwareMap.getMotor("intake2")

        firstMotor.setReversed()

        return MarkIIntake(firstMotor, secondMotor)
    }

    @JvmStatic
    fun makeIntakeFlippers(hardwareMap: HardwareMap): MarkIIntakeFlippers {
        val firstServo = hardwareMap.getServo("right_arm")
        val secondServo = hardwareMap.getServo("left_arm")

        return MarkIIntakeFlippers(firstServo, secondServo)
    }

    @JvmStatic
    fun makeFoundationMover(hardwareMap: HardwareMap): MarkIFoundationMover {
        val firstServo = hardwareMap.getServo("foundation_mover_1")
        val secondServo = hardwareMap.getServo("foundation_mover_2")
        secondServo.direction = Servo.Direction.REVERSE

        return MarkIFoundationMover(firstServo, secondServo)
    }

    @JvmStatic
    fun makeHardware(hardwareMap: HardwareMap): MarkIHardware {
        return MarkIHardware(
            makeDrive(hardwareMap),
            makeArm(hardwareMap),
            makeIntake(hardwareMap),
            makeIntakeFlippers(hardwareMap),
            makeFoundationMover(hardwareMap)
        )
    }
}

fun makeMarkIArm(hardwareMap: HardwareMap) = MarkIHardwareProvider.makeArm(hardwareMap)
fun makeMarkIIntake(hardwareMap: HardwareMap) = MarkIHardwareProvider.makeIntake(hardwareMap)
fun makeMarkIDrive(hardwareMap: HardwareMap) = MarkIHardwareProvider.makeDrive(hardwareMap)
fun makeMarkIHardware(hardwareMap: HardwareMap) = MarkIHardwareProvider.makeHardware(hardwareMap)