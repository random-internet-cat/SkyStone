package org.firstinspires.ftc.teamcode.hardware.provider

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.hardware.*
import org.firstinspires.ftc.teamcode.hardware.arm.MarkIArm
import org.firstinspires.ftc.teamcode.hardware.drive.*
import org.firstinspires.ftc.teamcode.hardware.drive.constants.MarkIDriveConstants
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.*
import org.firstinspires.ftc.teamcode.hardware.foundation_mover.MarkIFoundationMover
import org.firstinspires.ftc.teamcode.hardware.intake.MarkIIntake
import org.firstinspires.ftc.teamcode.hardware.intake_flippers.MarkIIntakeFlippers
import org.firstinspires.ftc.teamcode.hardware.rear_claw.MarkIRearClaws
import org.firstinspires.ftc.teamcode.util.*

object MarkIHardwareProvider {
    @JvmStatic
    fun makeDrive(hardwareMap: HardwareMap): MarkIDrivetrain {
        val imu = hardwareMap.getIMU()

        val frontLeft = hardwareMap.getMotorEx("front_left")
        val frontRight = hardwareMap.getMotorEx("front_right")
        val backLeft = hardwareMap.getMotorEx("back_left")
        val backRight = hardwareMap.getMotorEx("back_right")

        frontRight.setReversed()
        backRight.setReversed()

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

        return drive
    }

    @JvmStatic
    fun makeArmHorizontal(hardwareMap: HardwareMap): MarkIArm.HorizontalControl {
        val motor = hardwareMap.getMotor("horizontal")
        return MarkIArm.HorizontalControl(motor)
    }

    @JvmStatic
    fun makeArmVertical(hardwareMap: HardwareMap): MarkIArm.VerticalControl {
        val motor = hardwareMap.getMotor("vertical")
        motor.setReversed()
        motor.brakeOnZeroPower()
        motor.enableEncoder()
        return MarkIArm.VerticalControl(motor)
    }

    @JvmStatic
    fun makeArmClamp(hardwareMap: HardwareMap) = MarkIArm.Clamp(hardwareMap.getServo("clamp"))

    @JvmStatic
    fun makeArm(hardwareMap: HardwareMap): MarkIArm = MarkIArm(makeArmHorizontal(hardwareMap), makeArmVertical(hardwareMap), makeArmClamp(hardwareMap))

    @JvmStatic
    fun makeIntake(hardwareMap: HardwareMap): MarkIIntake {
        val firstMotor = hardwareMap.getMotor("intake")
        val secondMotor = hardwareMap.getMotor("intake2")

        firstMotor.setReversed()

        val firstServo = hardwareMap.getCRServo("left_intake_servo")
        val secondServo = hardwareMap.getCRServo("right_intake_servo")

        secondServo.direction = DcMotorSimple.Direction.REVERSE

        return MarkIIntake(listOf(firstMotor, secondMotor), listOf(firstServo, secondServo))
    }

    @JvmStatic
    fun makeIntakeFlippers(hardwareMap: HardwareMap): MarkIIntakeFlippers {
        val firstServo = hardwareMap.getCRServo("right_arm")
        firstServo.direction = DcMotorSimple.Direction.REVERSE

        val secondServo = hardwareMap.getCRServo("left_arm")

        return MarkIIntakeFlippers(firstServo, secondServo)
    }

    @JvmStatic
    fun makeFoundationMover(hardwareMap: HardwareMap): MarkIFoundationMover {
        val rightServo = hardwareMap.getServo("foundation_mover_1")
        val leftServo = hardwareMap.getServo("foundation_mover_2")
        leftServo.direction = Servo.Direction.REVERSE

        return MarkIFoundationMover(leftServo, rightServo)
    }

    @JvmStatic
    fun makeRearClaws(hardwareMap: HardwareMap): MarkIRearClaws {
        val right = hardwareMap.getServo("rear_clamp_left")
        val left = hardwareMap.getServo("rear_clamp_right")

        right.direction = Servo.Direction.REVERSE

        return MarkIRearClaws(left = left, right = right)
    }

    @JvmStatic
    fun makeHardware(hardwareMap: HardwareMap): MarkIHardware {
        return MarkIHardware(
            makeDrive(hardwareMap),
            makeArm(hardwareMap),
            makeIntake(hardwareMap),
            makeIntakeFlippers(hardwareMap),
            makeFoundationMover(hardwareMap),
            makeRearClaws(hardwareMap)
        )
    }
}

fun makeMarkIArm(hardwareMap: HardwareMap) = MarkIHardwareProvider.makeArm(hardwareMap)
fun makeMarkIIntake(hardwareMap: HardwareMap) = MarkIHardwareProvider.makeIntake(hardwareMap)
fun makeMarkIDrive(hardwareMap: HardwareMap) = MarkIHardwareProvider.makeDrive(hardwareMap)
fun makeMarkIHardware(hardwareMap: HardwareMap) = MarkIHardwareProvider.makeHardware(hardwareMap)