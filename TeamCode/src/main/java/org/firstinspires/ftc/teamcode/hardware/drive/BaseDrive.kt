package org.firstinspires.ftc.teamcode.hardware.drive

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.util.*
import kotlin.math.abs
import kotlin.math.max

data class BasicFourWheelDrivetrain<out MotorType : DcMotor>(val frontLeft: MotorType, val frontRight: MotorType, val backLeft: MotorType, val backRight: MotorType)

fun <MotorType : DcMotor> BasicFourWheelDrivetrain<MotorType>.motors() = listOf(frontLeft, frontRight, backLeft, backRight)
fun <MotorType : DcMotor> BasicFourWheelDrivetrain<MotorType>.leftMotors() = listOf(frontLeft, backLeft)
fun <MotorType : DcMotor> BasicFourWheelDrivetrain<MotorType>.rightMotors() = listOf(frontRight, backRight)

fun <MotorType : DcMotor> BasicFourWheelDrivetrain<MotorType>.forEachMotor(f: MotorType.() -> Unit) = motors().forEach(f)
fun <MotorType : DcMotor> BasicFourWheelDrivetrain<MotorType>.forEachLeftMotor(f: MotorType.() -> Unit) = leftMotors().forEach(f)
fun <MotorType : DcMotor> BasicFourWheelDrivetrain<MotorType>.forEachRightMotor(f: MotorType.() -> Unit) = rightMotors().forEach(f)

fun BasicFourWheelDrivetrain<*>.enableEncoders() = forEachMotor { resetEncoder() }
fun BasicFourWheelDrivetrain<*>.disableEncoders() = forEachMotor { disableEncoder() }

fun BasicFourWheelDrivetrain<*>.brakeOnZeroPower() = forEachMotor { brakeOnZeroPower() }
fun BasicFourWheelDrivetrain<*>.floatOnZeroPower() = forEachMotor { floatOnZeroPower() }

typealias FourWheelDrivetrain = BasicFourWheelDrivetrain<DcMotor>
typealias FourWheelDrivetrainEx = BasicFourWheelDrivetrain<DcMotorEx>

open class BasicBaseDrive<out MotorType : DcMotor>(val drivetrain: BasicFourWheelDrivetrain<MotorType>) {
    protected fun motors() = drivetrain.motors()
    protected fun leftMotors() = drivetrain.leftMotors()
    protected fun rightMotors() = drivetrain.rightMotors()

    protected fun forEachMotor(f: MotorType.() -> Unit) = drivetrain.forEachMotor(f)
    protected fun forEachLeftMotor(f: MotorType.() -> Unit) = drivetrain.forEachLeftMotor(f)
    protected fun forEachRightMotor(f: MotorType.() -> Unit) = drivetrain.forEachRightMotor(f)

    fun disableEncoders() = drivetrain.disableEncoders()

    fun brakeOnZeroPower() = drivetrain.brakeOnZeroPower()
    fun floatOnZeroPower() = drivetrain.floatOnZeroPower()

    fun power(power: Double) {
        require(-1 <= power && power <= 1)

        forEachMotor {
            setPower(power)
        }
    }

    fun leftPower(power: Double) {
        require(-1 <= power && power <= 1)

        forEachLeftMotor {
            setPower(power)
        }
    }

    fun rightPower(power: Double) {
        require(-1 <= power && power <= 1)

        forEachRightMotor {
            setPower(power)
        }
    }
}

inline fun BasicBaseDrive<*>.power(power: Int) {
    this.power(power.toDouble())
}

private val DEFAULT_DRIVE_POWER = 0.8

fun BasicBaseDrive<*>.driveForward(power: Double = DEFAULT_DRIVE_POWER) {
    requirePositivePower(power)

    this.power(power)
}

fun BasicBaseDrive<*>.driveBackward(power: Double = DEFAULT_DRIVE_POWER) {
    requirePositivePower(power)

    this.power(-power)
}

fun BasicBaseDrive<*>.stop() {
    this.power(0)
}

fun BasicBaseDrive<*>.arcadeDrive(linearPower: Double, turnPower: Double) {
    requireValidPower(linearPower)
    requireValidPower(turnPower)

    var left = linearPower - turnPower
    var right = linearPower + turnPower

    if (abs(left) > 1.0 || abs(right) > 1.0) {
        val bigger = max(abs(left), abs(right))
        left /= bigger
        right /= bigger
    }

    leftPower(left)
    rightPower(right)
}


fun BasicBaseDrive<*>.cheesyDrive(linearPower: Double, turnPower: Double, turnInPlace: Boolean) {
    requireValidPower(linearPower)
    requireValidPower(turnPower)

    val adjustedLinearPower = if (turnInPlace) 0.0 else linearPower
    val adjustedTurnPower = if (turnInPlace) turnPower else (linearPower * turnPower)

    arcadeDrive(adjustedLinearPower, adjustedTurnPower)
}

typealias BaseDrive = BasicBaseDrive<DcMotor>
typealias BaseDriveEx = BasicBaseDrive<DcMotorEx>