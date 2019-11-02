package org.firstinspires.ftc.teamcode.hardware.drive

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.util.requirePositivePower
import org.firstinspires.ftc.teamcode.util.requireValidPower
import org.firstinspires.ftc.teamcode.util.resetEncoder
import kotlin.math.abs
import kotlin.math.max

data class BasicFourWheelDrivetrain<out MotorType : DcMotor>(val frontLeft: MotorType, val frontRight: MotorType, val backLeft: MotorType, val backRight: MotorType) {
    fun motors() = listOf(frontLeft, frontRight, backLeft, backRight)
    fun leftMotors() = listOf(frontLeft, backLeft)
    fun rightMotors() = listOf(frontRight, backRight)
}


typealias FourWheelDrivetrain = BasicFourWheelDrivetrain<DcMotor>
typealias FourWheelDrivetrainEx = BasicFourWheelDrivetrain<DcMotorEx>

open class BasicBaseDrive<out MotorType : DcMotor>(protected val drivetrain: BasicFourWheelDrivetrain<MotorType>) {
    protected fun motors() = drivetrain.motors()
    protected fun leftMotors() = drivetrain.leftMotors()
    protected fun rightMotors() = drivetrain.rightMotors()

    protected fun forEachMotor(f: MotorType.() -> Unit) {
        motors().forEach(f)
    }

    protected fun forEachLeftMotor(f: MotorType.() -> Unit) {
        leftMotors().forEach(f)
    }

    protected fun forEachRightMotor(f: MotorType.() -> Unit) {
        rightMotors().forEach(f)
    }

    fun enableEncoders() {
        forEachMotor {
            resetEncoder()
        }
    }

    fun disableEncoders() {
        forEachMotor {
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        }
    }

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