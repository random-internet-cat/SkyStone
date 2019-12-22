package org.firstinspires.ftc.teamcode.opmode.testing.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.config.ValueProvider
import com.acmerobotics.dashboard.config.variable.BasicVariable
import com.acmerobotics.dashboard.config.variable.CustomVariable
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.teamcode.drive.mecanum.RRMecanumDriveBase
import org.firstinspires.ftc.teamcode.hardware.drive.brakeOnZeroPower
import org.firstinspires.ftc.teamcode.hardware.drive.enableEncoders
import org.firstinspires.ftc.teamcode.hardware.provider.makeMarkIDrive
import org.firstinspires.ftc.teamcode.util.roadrunner.PIDCoefficients
import org.firstinspires.ftc.teamcode.util.roadrunner.RobotPosition
import org.firstinspires.ftc.teamcode.util.roadrunner.forward
import org.firstinspires.ftc.teamcode.util.roadrunner.roadrunner
import org.firstinspires.ftc.teamcode.util.units.*


/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Config
@Autonomous(group = "drive")
class FollowerPIDTuner : LinearOpMode() {
    private val dashboard = FtcDashboard.getInstance()
    private var catName: String? = null
    private var catVar: CustomVariable? = null

    private lateinit var drive: RRMecanumDriveBase

    private val TRANSLATIONAL_PID_VAR_NAME = "Translational PID"
    private val HEADING_PID_VAR_NAME = "Heading PID"

    private fun makePidVariable(get: () -> PIDCoefficients, set: (PIDCoefficients) -> Unit): CustomVariable {
        val pidVar = CustomVariable()

        pidVar.putVariable("kP", BasicVariable(object : ValueProvider<Double> {
            override fun get(): Double? {
                return get().kP
            }

            override fun set(value: Double?) {
                set(get().copy(kP = value!!))
            }
        }))
        pidVar.putVariable("kI", BasicVariable(object : ValueProvider<Double> {
            override fun get(): Double? {
                return get().kI
            }

            override fun set(value: Double?) {
                set(get().copy(kI = value!!))
            }
        }))
        pidVar.putVariable("kD", BasicVariable(object : ValueProvider<Double> {
            override fun get(): Double? {
                return get().kD
            }

            override fun set(value: Double?) {
                set(get().copy(kD = value!!))
            }
        }))

        return pidVar
    }

    private fun addPidVariable() {
        catName = javaClass.simpleName
        catVar = dashboard.configRoot.getVariable(catName) as CustomVariable?
        if (catVar == null) {
            // this should never happen...
            catVar = CustomVariable()
            dashboard.configRoot.putVariable(catName, catVar)

            RobotLog.w("Unable to find top-level category %s", catName)
        }

        val catVar = catVar!!

        catVar.putVariable(TRANSLATIONAL_PID_VAR_NAME, makePidVariable(get = { drive.translationalPID() }, set = { pid -> drive.setTranslationalPID(pid) }))
        catVar.putVariable(HEADING_PID_VAR_NAME, makePidVariable(get = { drive.headingPID() }, set = { pid -> drive.setHeadingPID(pid) }))
        dashboard.updateConfig()
    }

    private fun removePidVariable() {
        val catVar = catVar!!

        if (catVar.size() > 1) {
            catVar.removeVariable(TRANSLATIONAL_PID_VAR_NAME)
            catVar.removeVariable(HEADING_PID_VAR_NAME)
        } else {
            dashboard.configRoot.removeVariable(catName)
        }

        dashboard.updateConfig()
    }

    @Override
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        drive = makeMarkIDrive(hardwareMap).also { it.brakeOnZeroPower(); it.enableEncoders() }.roadrunner()

        addPidVariable()

        drive.poseEstimate = RobotPosition(-DISTANCE / 2, -DISTANCE / 2, AnglePoint.zero()).roadrunner()

        waitForStart()

        while (!isStopRequested()) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(DISTANCE)
                            .build()
            )
            drive.turnSync(Degrees(90.0))
        }

        removePidVariable()
    }

    companion object {
        var DISTANCE = Inches(28.0)
    }
}