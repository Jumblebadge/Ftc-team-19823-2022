package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.linearSlide;
import org.firstinspires.ftc.teamcode.subsystems.twoServoBucket;
import org.firstinspires.ftc.teamcode.utility.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.pipelines.cameraActivity;
import org.firstinspires.ftc.teamcode.navigation.goToPoint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;


    @Config
    @Autonomous(name="parkAuto", group="Linear Opmode")
    public class parkAuto extends LinearOpMode {

        OpenCvWebcam webcam;

        Localizer localizer;

        //Initialize FTCDashboard
        FtcDashboard dashboard;

        double distance = 0;
        double lastX, lastY;

        Pose2d pose;
        Pose2d desiredPose;
        Pose2d temp;

        goToPoint auto;

        List<LynxModule> allHubs;
        //IMU
        BNO055IMU IMU;

        public void runOpMode() {
            telemetry.addData("Status", "Initialized");
            cameraActivity webcamStuff = new cameraActivity(webcam,hardwareMap,telemetry);

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            webcamStuff.initCamera();
            telemetry.addLine("Waiting for start");

            //Calibrate the IMU
            //CHANGE TO ODO HEADING!
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            IMU = hardwareMap.get(BNO055IMU.class, "IMU");
            IMU.initialize(parameters);
            IMU.startAccelerationIntegration(new Position(), new Velocity(), 1000);

            //Initialize FTCDashboard telemetry
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            //Link all of our hardware to our hardwaremap
            //E = encoder, m1 = motor 1, m2 = motor 2
            AnalogInput mod1E = hardwareMap.get(AnalogInput.class, "mod1E");
            AnalogInput mod2E = hardwareMap.get(AnalogInput.class, "mod2E");
            AnalogInput mod3E = hardwareMap.get(AnalogInput.class, "mod3E");

            DcMotorEx mod1m1 = hardwareMap.get(DcMotorEx.class, "mod1m1");
            DcMotorEx mod2m1 = hardwareMap.get(DcMotorEx.class, "mod2m1");
            DcMotorEx mod3m1 = hardwareMap.get(DcMotorEx.class, "mod3m1");

            DcMotorEx mod1m2 = hardwareMap.get(DcMotorEx.class, "mod1m2");
            DcMotorEx mod2m2 = hardwareMap.get(DcMotorEx.class, "mod2m2");
            DcMotorEx mod3m2 = hardwareMap.get(DcMotorEx.class, "mod3m2");

            VoltageSensor vSensor = hardwareMap.voltageSensor.iterator().next();

            mod1m2.setDirection(DcMotorSimple.Direction.REVERSE);
            mod2m2.setDirection(DcMotorSimple.Direction.REVERSE);

            //Bulk sensor reads
            allHubs = hardwareMap.getAll(LynxModule.class);

            //Initialize FTCDashboard
            dashboard = FtcDashboard.getInstance();

            //Create objects for the classes we use
            SwerveDrive drive = new SwerveDrive(telemetry, null, hardwareMap, true);
            auto = new goToPoint(drive,telemetry,dashboard);
            //Bulk sensor reads
            for (LynxModule module : allHubs) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }

            //Fast loop go brrr
            PhotonCore.enable();
            localizer = new TwoWheelTrackingLocalizer(hardwareMap,IMU);

            drive.setModuleAdjustments(0,-30,50);

            while (!isStarted() && !isStopRequested()) {
                webcamStuff.detectTags();
                sleep(20);
            }
            webcamStuff.closeCamera();
            AprilTagDetection detectedTag = webcamStuff.sideDetected();

            //auto starting
            if(detectedTag == null || detectedTag.id == 2) {
                runPoint(39,0,0);
            }
            else if(detectedTag.id == 1){
                runPoint(26,0,0);
                runPoint(26,26,0);
                runPoint(39,26,0);
            }
            else if(detectedTag.id == 3){
                runPoint(26,0,0);
                runPoint(26,-26,0);
                runPoint(39,-26,0);
            }

            if(detectedTag != null)
            {
                telemetry.addLine("tag:\n");
                webcamStuff.tagToTelemetry(detectedTag);
            }
            else
            {
                telemetry.addLine("never seen tag");
            }
            telemetry.update();

        }

        public void runPoint(double x, double y, double heading) {
            desiredPose = new Pose2d(x,y,heading);
            localizer.update();
            pose = localizer.getPoseEstimate();
            Pose2d startPose = pose;
            auto.driveToPoint(pose,desiredPose,startPose,true);
            while(!auto.isDone() && opModeIsActive()){
                for (LynxModule hub : allHubs) {
                    hub.clearBulkCache();
                }
                localizer.update();
                pose = localizer.getPoseEstimate();
                auto.driveToPoint(pose,desiredPose,startPose,false);
                telemetry.update();
            }
        }
    }

