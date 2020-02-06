package org.firstinspires.ftc.teamcode.hardware.provider

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotor
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
import org.firstinspires.ftc.teamcode.hardware.auto_claw.MarkIAutoClaws
import org.firstinspires.ftc.teamcode.hardware.drive.motors.withoutEncoderAccess
import org.firstinspires.ftc.teamcode.util.*
import org.firstinspires.ftc.teamcode.util.roadrunner.RobotPosition
import org.firstinspires.ftc.teamcode.util.roadrunner.roadrunner
import org.firstinspires.ftc.teamcode.util.units.*
import org.firstinspires.ftc.teamcode.util.units.Millimeters

object MarkIHardwareProvider {
    private val ODO_POD_CIRCUMFERENCE = Millimeters(35)

    private fun odometryPodTicksToPosition(motor: DcMotor): Distance {
        return ODO_POD_CIRCUMFERENCE * (motor.currentPosition / motor.motorType.ticksPerRev)
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

        val externalGearing = 1.0

        val drivetrain = MecanumDrivetrain(
            frontLeft = TypedMotorEx(frontLeft.withoutEncoderAccess(), externalGearing = externalGearing),
            frontRight = TypedMotorEx(frontRight.withoutEncoderAccess(), externalGearing = externalGearing),
            backLeft = TypedMotorEx(backLeft.withoutEncoderAccess(), externalGearing = externalGearing),
            backRight = TypedMotorEx(backRight.withoutEncoderAccess(), externalGearing = externalGearing)
        )

        data class OdoPodDesc(val pose: Pose2d, val motor: DcMotor) {
            constructor(pose: RobotPosition, motor: DcMotor) : this(pose.roadrunner(), motor)
        }

        val odoParallelDistance = MarkIOdomtetryConstants.PARALLEL_DISTANCE_IN

        val podDescs = listOf(
            OdoPodDesc(RobotPosition(Distance.zero(), Inches(odoParallelDistance / 2.0), DegreesPoint(0.0)).roadrunner(), frontRight),
            OdoPodDesc(RobotPosition(Distance.zero(), Inches(-1 * odoParallelDistance / 2.0), DegreesPoint(0.0)).roadrunner(), frontLeft),
            OdoPodDesc(RobotPosition(Inches(MarkIOdomtetryConstants.BACK_DISTANCE_IN), Distance.zero(), DegreesPoint(90.0)).roadrunner(), backLeft)
        )

        val drive = MecanumDrive(MecanumNoWheelEncoders(object : ThreeTrackingWheelLocalizer(
            podDescs.map { it.pose }
        ) {
            override fun getWheelPositions(): List<Double> {
                return podDescs.map { odometryPodTicksToPosition(it.motor).roadrunner().raw }
            }
        }), MarkIDriveConstants, drivetrain)

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

        return MarkIIntake(listOf(firstMotor, secondMotor))
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
    fun makeAutoClaws(hardwareMap: HardwareMap): MarkIAutoClaws {
        val right = hardwareMap.getServo("auto_claw_left")
        val left = hardwareMap.getServo("auto_claw_right")

        right.direction = Servo.Direction.REVERSE

        return MarkIAutoClaws(left = left, right = right)
    }

    @JvmStatic
    fun makeHardware(hardwareMap: HardwareMap): MarkIHardware {
        return MarkIHardware(
            makeDrive(hardwareMap),
            makeArm(hardwareMap),
            makeIntake(hardwareMap),
            makeIntakeFlippers(hardwareMap),
            makeFoundationMover(hardwareMap),
            makeAutoClaws(hardwareMap)
        )
    }
}

fun makeMarkIArm(hardwareMap: HardwareMap) = MarkIHardwareProvider.makeArm(hardwareMap)
fun makeMarkIIntake(hardwareMap: HardwareMap) = MarkIHardwareProvider.makeIntake(hardwareMap)
fun makeMarkIDrive(hardwareMap: HardwareMap) = MarkIHardwareProvider.makeDrive(hardwareMap)
fun makeMarkIHardware(hardwareMap: HardwareMap) = MarkIHardwareProvider.makeHardware(hardwareMap)