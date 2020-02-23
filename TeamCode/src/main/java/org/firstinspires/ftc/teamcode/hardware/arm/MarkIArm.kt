package org.firstinspires.ftc.teamcode.hardware.arm

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.*
import org.firstinspires.ftc.teamcode.util.roadrunner.PIDFCoefficients
import org.firstinspires.ftc.teamcode.util.units.*

data class MarkIArm(val horizontal: HorizontalControl, val vertical: VerticalControl, val clamp: Clamp) {
    data class HorizontalControl(val motor: DcMotorEx) {
        companion object {
            private const val MOTOR_POWER = 0.8
            private const val MIN_ENCODER_VALUE = 10
            private const val MAX_ENCODER_VALUE = 1255

            private const val MOVE_OUT_OVERSHOOT = 5
        }

        private enum class AutomaticState {
            IN {
                override fun runOnMotor(motor: DcMotor) {
                    motor.power = 0.0
                    motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

                    if (motor.currentPosition >= MIN_ENCODER_VALUE) {
                        motor.power = -1 * MOTOR_POWER
                    }
                }

                override fun shouldStopRunning(motor: DcMotor): Boolean {
                    return motor.currentPosition <= MIN_ENCODER_VALUE
                }
            },
            OUT {
                override fun runOnMotor(motor: DcMotor) {
                    motor.power = MOTOR_POWER
                    motor.targetPosition = MAX_ENCODER_VALUE + MOVE_OUT_OVERSHOOT
                    motor.mode = DcMotor.RunMode.RUN_TO_POSITION
                }

                override fun shouldStopRunning(motor: DcMotor): Boolean {
                    return motor.currentPosition >= MAX_ENCODER_VALUE
                }
            }
            ;

            abstract fun runOnMotor(motor: DcMotor)
            abstract fun shouldStopRunning(motor: DcMotor): Boolean
        }

        private var automaticState: AutomaticState? = null

        init {
            motor.resetEncoder()
        }

        fun power(rawPower: Double) {
            check(-1 <= rawPower && rawPower <= 1)

            switchToManual()

            val wouldExceedRetract = rawPower < 0 && motor.currentPosition <= MIN_ENCODER_VALUE
            val wouldExceedExtend = rawPower > 0 && motor.currentPosition >= MAX_ENCODER_VALUE
            val wouldExceedLimit = wouldExceedRetract || wouldExceedExtend

            val adjustedPower = if (wouldExceedLimit) 0.0 else rawPower

            motor.power = adjustedPower
        }

        private fun power(power: Int) = power(power.toDouble())

        fun moveOut() {
            power(MOTOR_POWER)
        }

        fun moveIn() {
            power(-MOTOR_POWER)
        }

        fun closeToIn() : Boolean {
            return (kotlin.math.abs(motor.currentPosition - MIN_ENCODER_VALUE) <= 20)
        }

        private fun moveToState(newState: AutomaticState) {
            newState.runOnMotor(motor)
            automaticState = newState
        }

        fun moveAllTheWayIn() = moveToState(AutomaticState.IN)
        fun moveAllTheWayOut() = moveToState(AutomaticState.OUT)

        fun pidf(): PIDFCoefficients = motor.pidf(DcMotor.RunMode.RUN_USING_ENCODER)

        fun setPIDF(pidf: PIDFCoefficients) {
            motor.setPIDF(DcMotor.RunMode.RUN_USING_ENCODER, pidf)
        }

        private fun isAutomatic() = automaticState != null
        private fun isManual() = !isAutomatic()

        private fun checkAutomatic() = automaticState ?: throw IllegalStateException()

        private fun switchToManual() {
            automaticState = null
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        fun stop() {
            power(0)
        }

        fun stopIfManual() {
            if (isManual()) stop()
        }

        private fun shouldStopAutomatic(): Boolean {
            val automaticState = checkAutomatic()
            return automaticState.shouldStopRunning(motor)
        }

        fun update() {
            // Check to turn off RunToPosition and reinstate manual control if target has been reached
            if (isAutomatic() && shouldStopAutomatic()) {
                stop()
                switchToManual()
            }
        }
    }

    @Config("MarkIArm VerticalControl")
    data class VerticalControl(private val motor: DcMotor) {
        companion object {
            private const val MANUAL_MOTOR_POWER = 0.6
            private const val AUTOMATIC_MOTOR_POWER = 0.9

            @JvmField
            public var _COLLECT_TICKS: Int = 0

            @JvmField
            public var _STAGE0_TICKS: Int = 370

            @JvmField
            public var _PER_STAGE_TICKS: Int = 690

            private const val MIN_ENCODER_TICKS = 10
            private const val MAX_ENCODER_TICKS = 4720

            private val COLLECT_POSITION get() = EncoderPosition(_COLLECT_TICKS)
            private val STAGE0_POSITION get() = EncoderPosition(_STAGE0_TICKS)
            private val PER_STAGE_TICKS get() = EncoderTicks(_PER_STAGE_TICKS)
        }

        sealed class State {
            companion object {
                private const val MIN_BLOCK_HEIGHT = 0
                private const val MAX_BLOCK_HEIGHT = 7
            }

            abstract val nextDown: State?
            abstract val nextUp: State?
            abstract val position: EncoderPosition

            object ZeroState : State() {
                override val nextDown: State? get() = null
                override val nextUp: State? get() = CollectState
                override val position: EncoderPosition
                    get() = EncoderPosition(MIN_ENCODER_TICKS)

                override fun toString(): String {
                    return "ZeroState"
                }
            }

            object CollectState : State() {
                override val nextDown: State? get() = null
                override val nextUp: State? get() = PlaceBlockState(MIN_BLOCK_HEIGHT)
                override val position: EncoderPosition
                    get() = COLLECT_POSITION

                override fun toString(): String {
                    return "CollectState"
                }
            }

            data class PlaceBlockState(val blockHeight: Int) : State() {
                init {
                    require(blockHeight >= MIN_BLOCK_HEIGHT && blockHeight <= MAX_BLOCK_HEIGHT)
                }

                override val nextDown: State?
                    get() = if (blockHeight == MIN_BLOCK_HEIGHT) CollectState else PlaceBlockState(blockHeight - 1)

                override val nextUp: State?
                    get() = if (blockHeight == MAX_BLOCK_HEIGHT) null else PlaceBlockState(blockHeight + 1)

                override val position: EncoderPosition
                    get() = STAGE0_POSITION + blockHeight * PER_STAGE_TICKS
            }
        }

        private var _state: State

        /**
         * @return The most recent (or current) automatic state, regardless of if currently in automatic
         */
        public fun mostRecentAutomaticState() = _state

        /**
         * @return If in automatic, the current state, otherwise null
         */
        public fun currentAutomaticState() = if (!isManual) _state else null

        init {
            motor.resetEncoder()
            motor.targetPosition = 0
            _state = State.CollectState
        }

        private var _mutableIsManual: Boolean = true

        private val isManual get() = _mutableIsManual
        private val isAutomatic get() = !isManual

        private fun manuallyMoveWithPower(logicalPower: Double) {
            val uncheckedRawPower = -logicalPower

            val motorPosition = motor.currentPosition
            val exceedsLowerLimit = uncheckedRawPower < 0 && motorPosition < MIN_ENCODER_TICKS
            val exceedsUpperLimit = uncheckedRawPower > 0 && motorPosition > MAX_ENCODER_TICKS
            val exceedsLimit = exceedsLowerLimit || exceedsUpperLimit

            val checkedRawPower = if (exceedsLimit) 0.0 else uncheckedRawPower

            markManual()
            setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER)
            motor.power = checkedRawPower
        }

        private fun markManual() {
            _mutableIsManual = true
        }

        private fun markAutomatic() {
            _mutableIsManual = false
        }

        fun manuallyMoveUp() {
            manuallyMoveWithPower(MANUAL_MOTOR_POWER)
        }

        fun manuallyMoveDown() {
            manuallyMoveWithPower(-MANUAL_MOTOR_POWER)
        }

        fun stop() {
            motor.power = 0.0
        }

        fun switchToManual() {
            manuallyMoveWithPower(0.0)
        }

        fun stopIfManual() {
            if (isManual) stop()
        }

        private fun setMotorRunMode(runMode: DcMotor.RunMode) {
            motor.mode = runMode
        }

        private fun moveToPosition(encoderPosition: EncoderPosition) {
            motor.targetPosition = encoderPosition.raw
            motor.power = AUTOMATIC_MOTOR_POWER
            setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION)
        }

        private fun setCurrentState(state: State) {
            markAutomatic()
            _state = state
        }

        private fun moveToCurrentState() {
            val currentState = currentAutomaticState()
            require(currentState != null) { "Request to move to state in manual mode" }
            moveToPosition(currentState.position)
        }

        public fun moveToZero() {
            moveToState(State.ZeroState)
        }

        private fun moveToCurrentStateIfAutomatic() {
            if (isAutomatic) moveToCurrentState()
        }

        fun moveToState(state: State) {
            setCurrentState(state)
            moveToCurrentState()
        }

        fun moveToCollect() = moveToState(State.CollectState)
        fun moveToPlace(height: Int) = moveToState(State.PlaceBlockState(height))

        fun isMovingToState(): Boolean {
            check(isAutomatic)
            check(motor.mode == DcMotor.RunMode.RUN_TO_POSITION)
            return motor.isBusy
        }

        fun update() {
            moveToCurrentStateIfAutomatic()
        }
    }

    @Config
    class Clamp(val servo: Servo) {
        companion object {
            @JvmField
            public var OPEN_POSITION = 0.5

            @JvmField
            public var CLOSED_POSITION = 0.8
        }

        fun open() {
            servo.position = OPEN_POSITION
        }

        fun close() {
            servo.position = CLOSED_POSITION
        }

        fun servoPosition() = servo.position

        fun update() {}
    }

    fun update() {
        horizontal.update()
        vertical.update()
        clamp.update()
    }
}