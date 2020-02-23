package org.firstinspires.ftc.teamcode.hardware.provider

import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.hardware.*
import org.firstinspires.ftc.teamcode.hardware.arm.MarkIArm
import org.firstinspires.ftc.teamcode.hardware.capstone_dropper.MarkICapstoneDropper
import org.firstinspires.ftc.teamcode.hardware.drive.*
import org.firstinspires.ftc.teamcode.hardware.drive.constants.MarkIDriveConstants
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.*
import org.firstinspires.ftc.teamcode.hardware.foundation_mover.MarkIFoundationMover
import org.firstinspires.ftc.teamcode.hardware.intake.MarkIIntake
import org.firstinspires.ftc.teamcode.hardware.drive.motors.withoutEncoderAccess
import org.firstinspires.ftc.teamcode.util.*
import org.firstinspires.ftc.teamcode.util.roadrunner.PIDFCoefficients
import org.firstinspires.ftc.teamcode.util.roadrunner.RobotPosition
import org.firstinspires.ftc.teamcode.util.roadrunner.roadrunner
import org.firstinspires.ftc.teamcode.util.units.*

object MarkIHardwareProvider {
    private fun odometryPodTicksToPosition(motor: DcMotor): Distance {
        return MarkIOdometryConstants.ODO_POD_CIRCUMFERENCE * (motor.currentPosition.toDouble() / MarkIOdometryConstants.POD_TICKS_PER_REV.toDouble())
    }

    @JvmStatic
    fun makeDrive(hardwareMap: HardwareMap): MarkIDrivetrain {
        val imu = hardwareMap.getIMU()

        val frontLeft = hardwareMap.getMotorEx("front_left")
        val frontRight = hardwareMap.getMotorEx("front_right")
        val backLeft = hardwareMap.getMotorEx("back_left")
        val backRight = hardwareMap.getMotorEx("back_right")

        frontRight.setReversed()
        backRight.setReversed()

        listOf(frontLeft, frontRight, backLeft, backRight).forEach { it.resetEncoder() }

        val externalGearing = 1.0

        val drivetrain = MecanumDrivetrain(
            frontLeft = TypedMotorEx(frontLeft, externalGearing = externalGearing),
            frontRight = TypedMotorEx(frontRight, externalGearing = externalGearing),
            backLeft = TypedMotorEx(backLeft, externalGearing = externalGearing),
            backRight = TypedMotorEx(backRight, externalGearing = externalGearing)
        )

        val drive = MecanumDrive(MecanumUseWheelEncoders(MecanumUseHeadingProvider(IMUHeadingProvider(imu))), MarkIDriveConstants, drivetrain)
        drive.enableEncoders()
        drive.brakeOnZeroPower()

        return drive
    }

    @JvmStatic
    fun makeArmHorizontal(hardwareMap: HardwareMap): MarkIArm.HorizontalControl {
        val motor = hardwareMap.getMotorEx("horizontal")
        return MarkIArm.HorizontalControl(motor).apply { setPIDF(PIDFCoefficients(12.0, 3.0, 0.0, 5.0)) }
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

        return MarkIIntake(listOf(firstMotor, secondMotor))
    }

    @JvmStatic
    fun makeFoundationMover(hardwareMap: HardwareMap): MarkIFoundationMover {
        val rightServo = hardwareMap.getServo("foundation_mover_1")
        val leftServo = hardwareMap.getServo("foundation_mover_2")
        leftServo.direction = Servo.Direction.REVERSE

        return MarkIFoundationMover(leftServo, rightServo)
    }

    @JvmStatic
    fun makeCapstoneDropper(hardwareMap: HardwareMap) : MarkICapstoneDropper {
        val capstoneDropper = hardwareMap.getServo("cap_mech")

        return MarkICapstoneDropper(capstoneDropper)
    }

    @JvmStatic
    fun makeHardware(hardwareMap: HardwareMap): MarkIHardware {
        return MarkIHardware(
            makeDrive(hardwareMap),
            makeArm(hardwareMap),
            makeIntake(hardwareMap),
            makeFoundationMover(hardwareMap),
            makeCapstoneDropper(hardwareMap)
        )
    }
}

fun makeMarkIArm(hardwareMap: HardwareMap) = MarkIHardwareProvider.makeArm(hardwareMap)
fun makeMarkIIntake(hardwareMap: HardwareMap) = MarkIHardwareProvider.makeIntake(hardwareMap)
fun makeMarkIDrive(hardwareMap: HardwareMap) = MarkIHardwareProvider.makeDrive(hardwareMap)
fun makeMarkIHardware(hardwareMap: HardwareMap) = MarkIHardwareProvider.makeHardware(hardwareMap)