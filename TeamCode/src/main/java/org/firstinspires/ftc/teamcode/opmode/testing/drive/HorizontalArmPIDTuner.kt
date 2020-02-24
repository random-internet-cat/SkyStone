package org.firstinspires.ftc.teamcode.opmode.testing.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.config.ValueProvider
import com.acmerobotics.dashboard.config.variable.BasicVariable
import com.acmerobotics.dashboard.config.variable.CustomVariable
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.teamcode.drive.mecanum.RRMecanumDriveBase
import org.firstinspires.ftc.teamcode.hardware.arm.MarkIArm
import org.firstinspires.ftc.teamcode.hardware.drive.brakeOnZeroPower
import org.firstinspires.ftc.teamcode.hardware.drive.constants.MarkIDriveConstants
import org.firstinspires.ftc.teamcode.hardware.drive.enableEncoders
import org.firstinspires.ftc.teamcode.hardware.drive.floatOnZeroPower
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.characterization
import org.firstinspires.ftc.teamcode.hardware.provider.makeMarkIDrive
import org.firstinspires.ftc.teamcode.hardware.provider.makeMarkIHardware
import org.firstinspires.ftc.teamcode.util.roadrunner.MotionState
import org.firstinspires.ftc.teamcode.util.roadrunner.PIDCoefficients
import org.firstinspires.ftc.teamcode.util.roadrunner.PIDFCoefficients
import org.firstinspires.ftc.teamcode.util.units.*

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
class HorizontalArmPIDTuner : LinearOpMode() {
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

    private fun addArmPidVariable(horizontalArm: MarkIArm.HorizontalControl) {
        addPidVariable(
            pidVarName = "Arm PID",
            getPIDF = { horizontalArm.pidf() },
            setPIDF = { pidf -> horizontalArm.setPIDF(pidf) }
        )
    }

    private fun removePidVariable() {
        catVar = null
        dashboard.configRoot.removeVariable(catName)
        dashboard.updateConfig()
    }

    private fun handleArmHorizontalInputs(gamepad: Gamepad, horizontal: MarkIArm.HorizontalControl) {
        val armState: String

        when {
            gamepad.left_trigger > 0.1 -> {
                horizontal.moveOut()
                armState = "Moving out"
            }

            gamepad.right_trigger > 0.1 -> {
                horizontal.moveIn()
                armState = "Moving in"
            }

            else -> {
                horizontal.stop()
                armState = "Not manually moving"
            }
        }

        telemetry.addData("Arm state", armState)
    }

    private fun handleArmAutomatedInputs(gamepad: Gamepad, horizontalArm: MarkIArm.HorizontalControl) {
        when {
            gamepad.b -> horizontalArm.moveAllTheWayIn()
        }
    }

    private fun handleArmInputs(gamepad: Gamepad, horizontalArm: MarkIArm.HorizontalControl) {
        handleArmHorizontalInputs(gamepad, horizontalArm)
        handleArmAutomatedInputs(gamepad, horizontalArm)
    }

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)
        telemetry.msTransmissionInterval = 15

        val hardware = makeMarkIHardware(hardwareMap)
        val horizontalArm = hardware.arm.horizontal

        addArmPidVariable(horizontalArm)

        waitForStart()

        while (opModeIsActive()) {
            handleArmInputs(gamepad1, horizontalArm)

            telemetry.update()
            hardware.update(updateDashboard = false)
        }

        removePidVariable()
    }
}