package org.firstinspires.ftc.teamcode.opmode.testing.arm

//import com.acmerobotics.dashboard.FtcDashboard
//import com.acmerobotics.dashboard.config.Config
//import com.acmerobotics.dashboard.config.ValueProvider
//import com.acmerobotics.dashboard.config.variable.BasicVariable
//import com.acmerobotics.dashboard.config.variable.CustomVariable
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
//import com.acmerobotics.roadrunner.util.NanoClock
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
//import com.qualcomm.robotcore.hardware.DcMotorEx
//import com.qualcomm.robotcore.util.RobotLog
//import org.firstinspires.ftc.teamcode.hardware.arm.PrototypeArm
//import org.firstinspires.ftc.teamcode.hardware.arm.RRArmRotator
//import org.firstinspires.ftc.teamcode.hardware.provider.makeMarkIArm
//import org.firstinspires.ftc.teamcode.hardware.provider.makeMarkIDrive
//import org.firstinspires.ftc.teamcode.util.encoderPosition
//import org.firstinspires.ftc.teamcode.util.units.*
//import org.firstinspires.ftc.teamcode.util.units.RadiansPerSecond
//
//@Config
//@Autonomous
//class ArmEncoderTest : LinearOpMode() {
//    override fun runOpMode() {
//        val arm = makeMarkIArm(hardwareMap).rotator
//        val armMotor = arm.motor
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
//        while (opModeIsActive()) {
//            telemetry.addData("Actual enc. position", armMotor.encoderPosition().raw)
//
//            val currentAngle = arm.currentAngle()
//            telemetry.addData("Arm angle (deg)", DegreesPoint(currentAngle).raw)
//
//            val currentTime = Seconds(clock.seconds())
//
//            if (lastAngle != null && lastTime != null) {
//                telemetry.addData("Angular velocity (deg/s)", DegreesPerSecond((currentAngle - lastAngle) / (currentTime - lastTime)).raw)
//            }
//
//            lastTime = currentTime
//            lastAngle = currentAngle
//
//            telemetry.update()
//        }
//    }
//}