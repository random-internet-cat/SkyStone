package org.firstinspires.ftc.teamcode.opmode.testing.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.config.ValueProvider
import com.acmerobotics.dashboard.config.variable.BasicVariable
import com.acmerobotics.dashboard.config.variable.CustomVariable
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.teamcode.hardware.arm.MarkIArm
import org.firstinspires.ftc.teamcode.hardware.provider.makeMarkIHardware
import org.firstinspires.ftc.teamcode.util.roadrunner.PIDFCoefficients

/*
 * This routine is designed to tune the PID coefficients used by the REV Expansion Hubs for closed-
 * loop velocity control. Altx`hough it may seem unnecessary, tuning these coefficients is just as
 * important as the positional parameters. Like the other manual tuning routines, this op mode
 * relies heavily upon the dashboard. To access the dashboard, connect your computer to the RC's
 * WiFi network and navigate to https://192.168.49.1:8080/dash in your browser. Once you've
 * successfully connected, start the program, and your robot will begin moving forward and backward
 * according to a motion profile. Your job is to graph the velocity errors over time and adjust the
 * PID coefficients. Once you've found a satisfactory set of gains, add them to your drive class
 * ctor.
 */
@Config
@Autonomous(group = "drive")
class VerticalArmPIDTuner : LinearOpMode() {
    private val dashboard = FtcDashboard.getInstance()
    private val catName: String = javaClass.simpleName
    private var catVar: CustomVariable? = null

    private fun addPidVariable(pidVarName: String, getPIDF: () -> PIDFCoefficients, setPIDF: (PIDFCoefficients) -> Unit) {
        catVar = dashboard.configRoot.getVariable(catName) as CustomVariable?

        if (catVar == null) {
            // this should never happen...
            catVar = CustomVariable()
            dashboard.configRoot.putVariable(catName, catVar)

            RobotLog.w("Unable to find top-level category %s", catName)
        }

        val pidVar = CustomVariable()
        pidVar.putVariable("kP", BasicVariable(object : ValueProvider<Double> {
            override fun get(): Double? {
                return getPIDF().kP
            }

            override fun set(value: Double?) {
                setPIDF(getPIDF().copy(kP = value!!))
            }
        }))

        pidVar.putVariable("kI", BasicVariable(object : ValueProvider<Double> {
            override fun get(): Double? {
                return getPIDF().kI
            }

            override fun set(value: Double?) {
                setPIDF(getPIDF().copy(kI = value!!))
            }
        }))

        pidVar.putVariable("kD", BasicVariable(object : ValueProvider<Double> {
            override fun get(): Double? {
                return getPIDF().kD
            }

            override fun set(value: Double?) {
                setPIDF(getPIDF().copy(kD = value!!))
            }
        }))

        pidVar.putVariable("kF", BasicVariable(object : ValueProvider<Double> {
            override fun get(): Double {
                return getPIDF().kF
            }

            override fun set(value: Double?) {
                setPIDF(getPIDF().copy(kF = value!!))
            }
        }))

        catVar!!.putVariable(pidVarName, pidVar)
        dashboard.updateConfig()
    }

    private fun addArmPidVariable(verticalArm: MarkIArm.VerticalControl) {
        addPidVariable(
            pidVarName = "Arm PID",
            getPIDF = { verticalArm.pidf() },
            setPIDF = { pidf -> verticalArm.setPIDF(pidf) }
        )
    }

    private fun removePidVariable() {
        catVar = null
        dashboard.configRoot.removeVariable(catName)
        dashboard.updateConfig()
    }

    private var _armMotionLastTick = false

    private fun handleArmVerticalInputs(gamepad: Gamepad, vertical: MarkIArm.VerticalControl) {
        val manualDirection = gamepad.left_stick_y

        when {
            gamepad.right_bumper -> {
                if (!_armMotionLastTick) {
                    val nextUpState = vertical.mostRecentAutomaticState().nextUp
                    if (nextUpState != null) vertical.moveToState(nextUpState)
                }

                _armMotionLastTick = true
            }

            gamepad.left_bumper -> {
                if (!_armMotionLastTick) {
                    val nextDownState = vertical.mostRecentAutomaticState().nextDown
                    if (nextDownState != null) vertical.moveToState(nextDownState)
                }

                _armMotionLastTick = true
            }

            manualDirection > 0.1 -> {
                vertical.manuallyMoveUp()
            }

            manualDirection < -0.1 -> {
                vertical.manuallyMoveDown()
            }

            else -> {
                vertical.stopIfManual()
                _armMotionLastTick = false
            }
        }
    }

    private fun handleArmInputs(gamepad: Gamepad, vertical: MarkIArm.VerticalControl) {
        handleArmVerticalInputs(gamepad, vertical)
    }

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)
        telemetry.msTransmissionInterval = 15

        val hardware = makeMarkIHardware(hardwareMap)
        val vertical = hardware.arm.vertical

        addArmPidVariable(vertical)

        waitForStart()

        while (opModeIsActive()) {
            handleArmInputs(gamepad1, vertical)

            telemetry.addData("Encoder position", vertical.encoderPosition().raw)

            telemetry.update()
            hardware.update(updateDashboard = false)
        }

        removePidVariable()
    }
}