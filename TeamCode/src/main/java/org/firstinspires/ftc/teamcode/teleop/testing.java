package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileBuilder;
import com.acmerobotics.roadrunner.profile.MotionSegment;
import com.acmerobotics.roadrunner.profile.AccelerationConstraint;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.maths.controlLoopMath;
import org.firstinspires.ftc.teamcode.maths.mathsOperations;

@Config
@Disabled
@TeleOp(name="testing", group="Linear Opmode")
public class testing extends LinearOpMode {

    FtcDashboard dashboard;
    public static double clawRotTarget = 0.5;
    public static double clawGrab = 0;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ServoImplEx clawrotL = hardwareMap.get(ServoImplEx.class,"clawrotL");
        ServoImplEx clawrotR = hardwareMap.get(ServoImplEx.class,"clawrotR");
        ServoImplEx claw = hardwareMap.get(ServoImplEx.class,"claw");
        claw.setPwmRange(new PwmControl.PwmRange(500,2500));
        dashboard = FtcDashboard.getInstance();

        PhotonCore.enable();

        waitForStart();
        while (opModeIsActive()) {


            clawrotL.setPosition(clawRotTarget);
            clawrotR.setPosition(1-clawRotTarget);
            claw.setPosition(clawGrab);


            telemetry.update();
        }

    }

}