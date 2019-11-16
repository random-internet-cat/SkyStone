package org.firstinspires.ftc.teamcode.hardware.arm

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.hardware.arm.PrototypeArm.Wrist.Companion.COLLECT_POSITION
import org.firstinspires.ftc.teamcode.hardware.arm.PrototypeArm.Wrist.Companion.STAGE0_POSITION
import org.firstinspires.ftc.teamcode.hardware.arm.PrototypeArm.Wrist.Companion.STAGE1_POSITION
import org.firstinspires.ftc.teamcode.util.*
import org.firstinspires.ftc.teamcode.util.roadrunner.PIDCoefficients
import org.firstinspires.ftc.teamcode.util.units.*

private typealias RotatorPosition = Int
private typealias WristPosition = Double

class PrototypeArm(val rotator: Rotator, val wrist: Wrist, val clamp: Clamp) {
    class Rotator(private val motor: TypedMotor) {
        constructor(motor: DcMotor) : this(TypedMotor(motor, externalGearing = 1.0))

        companion object {
            @JvmField
            public var _COLLECT_ANGLE_DEG: Int = 0

            private val COLLECT_ANGLE get() = DegreesPoint(_COLLECT_ANGLE_DEG)

            @JvmField
            public var _STAGE0_ANGLE_DEG: Int = 90

            private val STAGE0_ANGLE get() = DegreesPoint(_STAGE0_ANGLE_DEG)

            @JvmField
            public var _STAGE1_ANGLE_DEG: Int = 110

            private val STAGE1_ANGLE get() = DegreesPoint(_STAGE1_ANGLE_DEG)
        }

        init {
            motor.disableEncoder()
            motor.resetEncoder()
        }

        fun moveToCollect() {
            roadrunner().moveToAngle(COLLECT_ANGLE)
        }

        fun moveToStage0() {
            roadrunner().moveToAngle(STAGE0_ANGLE)
        }

        fun moveToStage1() {
            roadrunner().moveToAngle(STAGE1_ANGLE)
        }

        private val roadrunnerValue by lazy {
            RRArmRotator(
                motor,
                PIDCoefficients(
                    kP = 0.0,
                    kI = 0.0,
                    kD = 0.0
                ),
                DcMotorCharacterization(kV = 0.45620, kA = 0.0, kStatic = 0.20775),
                0.001,
                ArmMovementConstraints(
                    maxAngle = RadiansPoint(TWO_PI),
                    maxVel = RevolutionsPerSecond(1.0),
                    maxAccel = RevolutionsPerSecondSquared(1.0),
                    maxJerk = RevolutionsPerSecondCubed(1.0)
                )
            )
        }

        fun roadrunner() = roadrunnerValue

        fun update() {
            roadrunner().update()
        }

        fun currentAngle() = roadrunner().currentAngle()
    }

    class Wrist(val servo: Servo) {
        companion object {
            @JvmField
            public var COLLECT_POSITION: WristPosition = 0.8

            @JvmField
            public var STAGE0_POSITION: WristPosition = 0.0

            @JvmField
            public var STAGE1_POSITION: WristPosition = 0.0
        }

        private fun setPosition(position: WristPosition) {
            servo.setPosition(position)
        }

        fun moveToCollect() {
            setPosition(COLLECT_POSITION)
        }

        fun moveToStage0() {
            setPosition(STAGE0_POSITION)
        }

        fun moveToStage1() {
            setPosition(STAGE1_POSITION)
        }
    }

    class Clamp(val servo: Servo) {
        companion object {
            @JvmField
            public var OPEN_POSITION = 0.7

            @JvmField
            public var CLOSED_POSITION = 0.98
        }

        fun open() {
            servo.position = OPEN_POSITION
        }

        fun close() {
            servo.position = CLOSED_POSITION
        }

        fun servoPosition() = servo.position
    }

    fun update() {
        rotator.update()
    }
}
