package org.firstinspires.ftc.teamcode.teleop;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.hardware.lynx.*;
import com.acmerobotics.dashboard.*;
import com.qualcomm.robotcore.util.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.navigation.GoToPoint;
import org.firstinspires.ftc.teamcode.utility.TwoWheelTrackingLocalizer;

import java.util.List;


@Config
@TeleOp(name="swervy", group="Linear Opmode")
public class swervy extends LinearOpMode {

    //Initialize FTCDashboard
    FtcDashboard dashboard;

    //Timers for the PID loops
    ElapsedTime hztimer = new ElapsedTime();

    //Tuning values so that wheels are always facing straight (accounts for encoder drift - tuned manually)
    public static double mod3PC = 20, mod1PC = 10, mod2PC = -20;

    //IMU
    Localizer localizer;
    public static double x=0,y=0, heading = 0,Kp=0,Kd=0,Ki=0, Kf = 0,hKp = 0,hKd = 0,hKi = 0, hKf = 0, maxVel=1,maxAccel=1,maxJerk=1, limit = 1000, hLimit = 1000;
    double lastX=0,lastY=0;
    Pose2d temp = new Pose2d(0,0,0);
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        //Initialize FTCDashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        //Initialize FTCDashboard
        dashboard = FtcDashboard.getInstance();

        //set odometry localizer and make object for driving
        localizer = new TwoWheelTrackingLocalizer(hardwareMap,imu);

        SwerveDrive drive = new SwerveDrive(telemetry, imu, hardwareMap, true);
        GoToPoint auto = new GoToPoint(drive,telemetry,dashboard);

        //Bulk sensor reads
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //Fast loop go brrr
        PhotonCore.enable();

        waitForStart();
        while (opModeIsActive()) {

            //Clear the cache for better loop times (bulk sensor reads)
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            localizer.update();
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            Pose2d pose = localizer.getPoseEstimate();
            Pose2d desiredPose = new Pose2d(x,y,heading);
            fieldOverlay.setStrokeWidth(1);
            fieldOverlay.setStroke("#3F51B5");
            drawRobot(fieldOverlay, pose);
            fieldOverlay.setStroke("#51B53F");
            drawRobot(fieldOverlay,desiredPose);
            fieldOverlay.setStroke("#B53F51");
            drawRobot(fieldOverlay,temp);

            packet.put("x", pose.getX());
            packet.put("y", pose.getY());
            packet.put("heading (deg)", Math.toDegrees(pose.getHeading()));

            dashboard.sendTelemetryPacket(packet);

            auto.setPIDCoeffs(Kp,Kd,Ki,Kf, limit);
            auto.setHeadingPIDcoeffs(hKp,hKd,hKi, hKf, hLimit);
            //auto.setProfileConstraints(maxVel,maxAccel,maxJerk);

            if (lastX != x || lastY != y) {
                lastX = x;
                lastY = y;
                temp = new Pose2d(pose.getX(), pose.getY(),pose.getHeading());
                auto.driveToPoint(pose,desiredPose,temp,true);
            }
            else{ lastX = x; lastY = y; }
            auto.driveToPoint(pose,desiredPose,temp,false);

            telemetry.addData("heading",pose.getHeading());
            telemetry.addData("targetx",desiredPose.getX());
            telemetry.addData("targety",desiredPose.getY());
            telemetry.addData("targetheading",desiredPose.getHeading());
            //telemetry.addData("gx",gamepad1.left_stick_x);
            //telemetry.addData("gy",gamepad1.left_stick_y);
            //telemetry.addData("headingout",rotPIDout);
            //telemetry.addData("headingtarget",headingTarget);
            //drivein.drive(gamepad1.left_stick_x,-gamepad1.left_stick_y,gamepad1.right_stick_x,gamepad1);

            //telemetry.setAutoClear(false);
            telemetry.addData("ms",hztimer.milliseconds());
            hztimer.reset();
            telemetry.update();
        }
    }

    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.getX(), pose.getY(), 9);
        Vector2d v = pose.headingVec().times(9);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }
}