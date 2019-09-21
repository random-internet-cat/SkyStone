package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor

open class BaseDrive {
    protected val m_frontLeft: DcMotor
    protected val m_frontRight: DcMotor
    protected val m_backLeft: DcMotor
    protected val m_backRight: DcMotor

    constructor(frontLeft: DcMotor, frontRight: DcMotor, backLeft: DcMotor, backRight: DcMotor) {
        m_frontLeft = frontLeft
        m_frontRight = frontRight
        m_backLeft = backLeft
        m_backRight = backRight
    }

    protected fun forEachMotor(f: DcMotor.() -> Unit) {
        m_frontLeft.f()
        m_frontRight.f()
        m_backLeft.f()
        m_backRight.f()
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
}

inline fun BaseDrive.power(power: Int) {
    this.power(power.toDouble())
}

private val DEFAULT_DRIVE_POWER = 0.8

fun BaseDrive.driveForward(power: Double = DEFAULT_DRIVE_POWER) {
    require(0 <= power && power <= 1)

    this.power(power)
}

fun BaseDrive.driveBackward(power: Double = DEFAULT_DRIVE_POWER) {
    require(0 <= power && power <= 1)

    this.power(-power)
}

fun BaseDrive.stop() {
    this.power(0)
}