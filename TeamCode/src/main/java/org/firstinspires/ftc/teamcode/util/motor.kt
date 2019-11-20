package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.teamcode.util.roadrunner.FtcPIDFCoefficients
import org.firstinspires.ftc.teamcode.util.roadrunner.PIDCoefficients
import org.firstinspires.ftc.teamcode.util.roadrunner.PIDFCoefficients
import org.firstinspires.ftc.teamcode.util.units.EncoderPosition
import org.firstinspires.ftc.teamcode.util.units.EncoderTicks
import org.firstinspires.ftc.teamcode.util.units.Radians
import org.firstinspires.ftc.teamcode.util.units.RadiansPoint

fun DcMotor.resetEncoder() {
    val oldMode = this.mode
    this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
    this.setMode(oldMode)
}

fun DcMotor.disableEncoder() {
    this.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
}

fun DcMotor.enableEncoder() {
    this.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
    resetEncoder()
}

fun DcMotor.setReversed() {
    this.setDirection(DcMotorSimple.Direction.REVERSE)
}

fun DcMotor.encoderPosition() = EncoderPosition(currentPosition)

fun DcMotorEx.setPID(runmode: DcMotor.RunMode, p: Double, i: Double, d: Double) {
    val f = getPIDFCoefficients(runmode).f
    setPIDFCoefficients(runmode, FtcPIDFCoefficients(p, i, d, f))
}

fun DcMotorEx.setPID(runmode: DcMotor.RunMode, pid: PIDCoefficients) {
    setPID(runmode, pid.kP, pid.kI, pid.kD)
}

fun DcMotorEx.setPIDF(runmode: DcMotor.RunMode, pidf: FtcPIDFCoefficients) {
    setPIDFCoefficients(runmode, pidf)
}

fun DcMotorEx.setPIDF(runmode: DcMotor.RunMode, p: Double, i: Double, d: Double, f: Double) {
    setPIDFCoefficients(runmode, FtcPIDFCoefficients(p, i, d, f))
}

fun DcMotorEx.setPIDF(runmode: DcMotor.RunMode, pidf: PIDFCoefficients) {
    setPIDF(runmode, pidf.kP, pidf.kI, pidf.kD, pidf.kF)
}

fun DcMotorEx.pid(runmode: DcMotor.RunMode): PIDCoefficients {
    val pidf = getPIDFCoefficients(runmode)
    return PIDCoefficients(pidf)
}

fun DcMotorEx.pidf(runmode: DcMotor.RunMode): PIDFCoefficients {
    val pidf = getPIDFCoefficients(runmode)
    return PIDFCoefficients(pidf)
}

fun DcMotor.brakeOnZeroPower() = setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
fun DcMotor.floatOnZeroPower() = setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)

fun requireValidPower(power: Double) {
    require(-1 <= power && power <= 1)
}

fun requirePositivePower(power: Double) {
    require(0 <= power && power <= 1)
}

data class MotorConfiguration(val rawConfig: MotorConfigurationType, val externalGearing: Double) {
    init {
        require(rawConfig != MotorConfigurationType.getUnspecifiedMotorType())
    }
}

fun MotorConfiguration.encoderToAngle(ticks: EncoderTicks) = Radians(TWO_PI * externalGearing * ticks.raw / rawConfig.ticksPerRev)
fun MotorConfiguration.encoderToAngle(ticks: EncoderPosition) = RadiansPoint(encoderToAngle(EncoderTicks(ticks.raw)).raw)

data class BasicTypedMotor<out MotorType : DcMotor>(val motor: MotorType, val config: MotorConfiguration) {
    constructor(motor: MotorType, externalGearing: Double) : this(motor, MotorConfiguration(motor.motorType, externalGearing))

    constructor(motor: MotorType, configType: Class<*>, externalGearing: Double) : this(motor, MotorConfiguration(MotorConfigurationType.getMotorType(configType), externalGearing))
}

typealias TypedMotor = BasicTypedMotor<DcMotor>
typealias TypedMotorEx = BasicTypedMotor<DcMotorEx>

fun BasicTypedMotor<*>.encoderPosition() = motor.encoderPosition()
fun BasicTypedMotor<*>.anglePosition() = config.encoderToAngle(encoderPosition())

fun BasicTypedMotor<*>.getPower() = motor.getPower()
fun BasicTypedMotor<*>.setPower(power: Double) = motor.setPower(power)
fun BasicTypedMotor<*>.setPower(power: Int) = setPower(power.toDouble())

fun BasicTypedMotor<*>.getMode() = motor.getMode()
fun BasicTypedMotor<*>.setMode(mode: DcMotor.RunMode) = motor.setMode(mode)

fun BasicTypedMotor<*>.getZeroPowerBehavior() = motor.getZeroPowerBehavior()
fun BasicTypedMotor<*>.setZeroPowerBehavior(behavior: DcMotor.ZeroPowerBehavior) = motor.setZeroPowerBehavior(behavior)

fun BasicTypedMotor<*>.brakeOnZeroPower() = motor.brakeOnZeroPower()
fun BasicTypedMotor<*>.floatOnZeroPower() = motor.floatOnZeroPower()

fun BasicTypedMotor<*>.enableEncoder() = motor.enableEncoder()
fun BasicTypedMotor<*>.disableEncoder() = motor.disableEncoder()
fun BasicTypedMotor<*>.resetEncoder() = motor.resetEncoder()

fun BasicTypedMotor<*>.getTargetPosition(): EncoderPosition = EncoderPosition(motor.getTargetPosition())
fun BasicTypedMotor<*>.setTargetPosition(value: Int) = motor.setTargetPosition(value)
fun BasicTypedMotor<*>.setTargetPosition(value: EncoderPosition) = setTargetPosition(value.raw)

fun BasicTypedMotor<*>.targetAnglePosition() = config.encoderToAngle(getTargetPosition())

data class DcMotorCharacterization(val kV: Double, val kA: Double, val kStatic: Double) {
    companion object {
        fun forBuiltinPID(kV: Double) = DcMotorCharacterization(kV = kV, kA = 0.0, kStatic = 0.0)
    }

    constructor() : this(0.0, 0.0, 0.0)
}