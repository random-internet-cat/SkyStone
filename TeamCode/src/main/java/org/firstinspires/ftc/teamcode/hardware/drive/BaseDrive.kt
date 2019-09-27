package org.firstinspires.ftc.teamcode.hardware.drive

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx

open class BasicBaseDrive<out MotorType : DcMotor> {
    protected val m_frontLeft: MotorType
    protected val m_frontRight: MotorType
    protected val m_backLeft: MotorType
    protected val m_backRight: MotorType

    constructor(frontLeft: MotorType, frontRight: MotorType, backLeft: MotorType, backRight: MotorType) {
        m_frontLeft = frontLeft
        m_frontRight = frontRight
        m_backLeft = backLeft
        m_backRight = backRight
    }

    fun motors() = listOf(m_frontLeft, m_frontRight, m_backLeft, m_backRight)
    fun leftMotors() = listOf(m_frontLeft, m_backLeft)
    fun rightMotors() = listOf(m_frontRight, m_backRight)

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
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
            setMode(DcMotor.RunMode.RUN_USING_ENCODER)
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
    require(0 <= power && power <= 1)

    this.power(power)
}

fun BasicBaseDrive<*>.driveBackward(power: Double = DEFAULT_DRIVE_POWER) {
    require(0 <= power && power <= 1)

    this.power(-power)
}

fun BasicBaseDrive<*>.stop() {
    this.power(0)
}

typealias BaseDrive = BasicBaseDrive<DcMotor>
typealias BaseDriveEx = BasicBaseDrive<DcMotorEx>