package org.firstinspires.ftc.teamcode.opmode.testing.arm

//import android.widget.Button
//import com.acmerobotics.dashboard.FtcDashboard
//import com.acmerobotics.dashboard.config.Config
//import com.acmerobotics.dashboard.config.ValueProvider
//import com.acmerobotics.dashboard.config.variable.BasicVariable
//import com.acmerobotics.dashboard.config.variable.CustomVariable
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
//import com.acmerobotics.roadrunner.util.NanoClock
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
//import com.qualcomm.robotcore.hardware.DcMotor
//import com.qualcomm.robotcore.hardware.DcMotorEx
//import com.qualcomm.robotcore.util.RobotLog
//import org.firstinspires.ftc.teamcode.hardware.arm.MarkIArm
//import org.firstinspires.ftc.teamcode.hardware.arm.PrototypeArm
//import org.firstinspires.ftc.teamcode.hardware.arm.RRArmRotator
//import org.firstinspires.ftc.teamcode.hardware.provider.makeMarkIArm
//import org.firstinspires.ftc.teamcode.util.*
//import org.firstinspires.ftc.teamcode.util.roadrunner.PIDCoefficients
//import org.firstinspires.ftc.teamcode.util.units.*
//import org.firstinspires.ftc.teamcode.util.units.RadiansPerSecond
//
//@Config
//@Autonomous
//class ArmRRPIDTuner : LinearOpMode() {
//    private val dashboard = FtcDashboard.getInstance()
//    private var catName: String = javaClass.simpleName
//    private var catVar: CustomVariable? = null
//
//    companion object {
//        private const val PID_VAR_NAME = "PID"
//    }
//
//    private fun addPidVariable(arm: RRArmRotator) {
//        catVar = dashboard.configRoot.getVariable(catName) as CustomVariable?
//
//        if (catVar == null) {
//            // this should never happen...
//            catVar = CustomVariable()
//            dashboard.configRoot.putVariable(catName, catVar)
//
//            RobotLog.w("Unable to find top-level category %s", catName)
//        }
//
//        val pidVar = CustomVariable()
//
//        pidVar.putVariable("kP", BasicVariable(object : ValueProvider<Double> {
//            override fun get(): Double {
//                return arm.pid().kP
//            }
//
//            override fun set(value: Double) {
//                arm.setPID(arm.pid().copy(kP = value))
//            }
//        }))
//
//        pidVar.putVariable("kI", BasicVariable(object : ValueProvider<Double> {
//            override fun get(): Double {
//                return arm.pid().kI
//            }
//
//            override fun set(value: Double) {
//                arm.setPID(arm.pid().copy(kI = value))
//            }
//        }))
//
//        pidVar.putVariable("kD", BasicVariable(object : ValueProvider<Double> {
//            override fun get(): Double {
//                return arm.pid().kD
//            }
//
//            override fun set(value: Double) {
//                arm.setPID(arm.pid().copy(kD = value))
//            }
//        }))
//
//        pidVar.putVariable("angleF", BasicVariable(object : ValueProvider<Double> {
//            override fun get(): Double {
//                return arm.angleFeedforward()
//            }
//
//            override fun set(value: Double) {
//                arm.setAngleFeedforward(value)
//            }
//        }))
//
//        catVar?.putVariable(PID_VAR_NAME, pidVar)
//        dashboard.updateConfig()
//    }
//
//    private fun removePidVariable() {
//        val catVar = catVar
//        if (catVar != null && catVar.size() > 1) {
//            catVar.removeVariable(PID_VAR_NAME)
//        } else {
//            dashboard.configRoot.removeVariable(catName)
//        }
//
//        dashboard.updateConfig()
//    }
//
//    override fun runOpMode() {
//        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)
//        telemetry.msTransmissionInterval = 10
//
//        val arm = makeMarkIArm(hardwareMap).rotator
//        val armMotor = arm.motor
//
//        arm.roadrunner().setAngleFeedforward(0.0)
//        arm.roadrunner().setPID(PIDCoefficients(0, 0, 0))
//
//        addPidVariable(arm.roadrunner())
//
//        telemetry.log().add("Ready!")
//        telemetry.update()
//        telemetry.clearAll()
//
//        waitForStart()
//
//        val clock = NanoClock.system()
//
//        var lastAngle: RadiansPoint? = null
//        var lastTime: Seconds? = null
//
//        var isForward: Boolean = false
//
//        while (opModeIsActive()) {
////            when {
////                gamepad1.a -> arm.moveToCollect()
////                gamepad1.b -> arm.moveToStage0()
////                gamepad1.y -> arm.moveToStage1()
////            }
//
//            val button = gamepad1.x
//
//            if (button) {
//                if (isForward && arm.currentAngle() >= DegreesPoint(170)) {
//                    isForward = false
//                    arm.roadrunner().moveToAngle(DegreesPoint(5))
//                }
//
//                if (!isForward && arm.currentAngle() <= DegreesPoint(10)) {
//                    isForward = true
//                    arm.roadrunner().moveToAngle(DegreesPoint(175))
//                }
//            }
//
//            telemetry.addData("Actual enc. position", armMotor.encoderPosition().raw)
//
//            val currentAngle = arm.currentAngle()
//            telemetry.addData("Arm angle (deg)", DegreesPoint(currentAngle).raw)
//
//            val targetAngle = arm.roadrunner().targetAngle()
//            if (targetAngle != null) telemetry.addData("Arm target (deg)", DegreesPoint(targetAngle).raw)
//
//            val currentTime = Seconds(clock.seconds())
//
//            if (lastAngle != null && lastTime != null) {
//                telemetry.addData("Angular velocity (deg/s)", DegreesPerSecond((currentAngle - lastAngle) / (currentTime - lastTime)).raw)
//            }
//
//            telemetry.addData("Motor power", armMotor.getPower())
//
//            lastTime = currentTime
//            lastAngle = currentAngle
//
//            telemetry.update()
//            arm.update()
//        }
//
//        removePidVariable()
//    }
//}