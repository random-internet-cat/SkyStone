package org.firstinspires.ftc.teamcode.hardware.drive.motors

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

class BadEncoderAccessException: Exception("Request for encoder on motor without encoder access")

open class NoEncoderDcMotor(val motor: DcMotor) : DcMotor by motor {
    init {
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    protected fun badAccess(): Nothing = throw BadEncoderAccessException()

    override fun setTargetPosition(position: Int) {
        badAccess()
    }

    override fun getTargetPosition(): Int {
        badAccess()
    }

    override fun getCurrentPosition(): Int {
        badAccess()
    }

    override fun setMode(mode: DcMotor.RunMode) {
        val migrated = mode.migrate()
        require(migrated == DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    }
}

class NoEncoderDcMotorEx(val motor: DcMotorEx) : DcMotorEx by motor {
    init {
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    protected fun badAccess(): Nothing = throw BadEncoderAccessException()

    override fun setTargetPosition(position: Int) {
        badAccess()
    }

    override fun getTargetPosition(): Int {
        badAccess()
    }

    override fun getCurrentPosition(): Int {
        badAccess()
    }

    override fun setMode(mode: DcMotor.RunMode) {
        val migrated = mode.migrate()
        require(migrated == DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    }

    override fun setVelocity(angularRate: Double) {
        badAccess()
    }

    override fun setVelocity(angularRate: Double, unit: AngleUnit?) {
        badAccess()
    }

    override fun getVelocity(): Double {
        badAccess()
    }

    override fun getVelocity(unit: AngleUnit?): Double {
        badAccess()
    }

    override fun setPIDCoefficients(mode: DcMotor.RunMode?, pidCoefficients: PIDCoefficients?) {
        badAccess()
    }

    override fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double) {
        badAccess()
    }

    override fun setPositionPIDFCoefficients(p: Double) {
        badAccess()
    }

    override fun getPIDCoefficients(mode: DcMotor.RunMode?): PIDCoefficients {
        badAccess()
    }

    override fun getPIDFCoefficients(mode: DcMotor.RunMode?): PIDFCoefficients {
        badAccess()
    }

    override fun setTargetPositionTolerance(tolerance: Int) {
        badAccess()
    }

    override fun getTargetPositionTolerance(): Int {
        badAccess()
    }
}

fun DcMotor.withoutEncoderAccess() = NoEncoderDcMotor(this)
fun DcMotorEx.withoutEncoderAccess() = NoEncoderDcMotorEx(this)