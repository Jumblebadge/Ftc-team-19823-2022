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

import org.firstinspires.ftc.teamcode.utility.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.pipelines.cameraActivity;
import org.firstinspires.ftc.teamcode.navigation.goToPoint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    double cyclesCompleted = 0;

    enum apexStates {
        DRIVE_TO_CYCLE,
        CYCLING,
        PARK
    }

    enum cycleStates {
        INTAKE,
        DEPOSIT,
        WAIT
    }

    apexStates apexstate = apexStates.DRIVE_TO_CYCLE;
    cycleStates cyclestate = cycleStates.WAIT;

    Pose2d pose;
    Pose2d desiredPose;
    Pose2d temp;

    goToPoint auto;

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
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        //Initialize FTCDashboard
        dashboard = FtcDashboard.getInstance();

        //Create objects for the classes we use
        SwerveDrive drive = new SwerveDrive(telemetry,mod1m1,mod1m2,mod2m1,mod2m2,mod3m1,mod3m2,mod1E,mod2E,mod3E,IMU,allHubs,vSensor, false);
        auto = new goToPoint(drive,telemetry,dashboard);
        //Bulk sensor reads
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //Fast loop go brrr
        PhotonCore.enable();
        localizer = new TwoWheelTrackingLocalizer(hardwareMap,IMU);

        drive.setModuleAdjustments(0,-15,-45);

        while (!isStarted() && !isStopRequested()) {
            webcamStuff.detectTags();
            sleep(20);
        }
        webcamStuff.closeCamera();
        AprilTagDetection detectedTag = webcamStuff.sideDetected();

        while (opModeIsActive()) {
            //Clear the cache for better loop times (bulk sensor reads)
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            runPoint(desiredPose.getX(),desiredPose.getY(),desiredPose.getHeading());

            switch (apexstate){
                case DRIVE_TO_CYCLE:
                    //drive to cycling position
                    desiredPose = new Pose2d(0,0,0);
                    apexstate = apexStates.CYCLING;
                    break;

                case CYCLING:
                    if (cyclesCompleted == 0){
                        cyclestate = cycleStates.DEPOSIT;
                    }
                    else if (cyclesCompleted == 6){
                        apexstate = apexStates.PARK;
                        cyclestate = cycleStates.WAIT;
                    }
                    break;

                case PARK:
                    //drive to park position
                    if(detectedTag == null || detectedTag.id == 2) {
                        desiredPose = new Pose2d(39,0,0);
                        desiredPose = new Pose2d(39,6,0);
                    }
                    else if(detectedTag.id == 1){
                        desiredPose = new Pose2d(26,0,0);
                        desiredPose = new Pose2d(26,-26,0);
                        desiredPose = new Pose2d(39,-30,0);
                    }
                    else if(detectedTag.id == 3){
                        desiredPose = new Pose2d(26,0,0);
                        desiredPose = new Pose2d(26,26,0);
                        desiredPose = new Pose2d(39,30,0);
                    }
                    break;
            }

            switch (cyclestate){
                case WAIT:
                    //set all systems to init position
                case INTAKE:
                    //intake extends and then retracts with cone, then drops into deposit box
                    cyclestate = cycleStates.DEPOSIT;
                    break;

                case DEPOSIT:
                    //lift slides drop cone come back down
                    cyclestate = cycleStates.INTAKE;
                    cyclesCompleted += 1;
                    break;
            }
        }
        //auto starting
        if(detectedTag == null || detectedTag.id == 2) {
            runPoint(39,0,0);
            runPoint(39,6,0);
        }
        else if(detectedTag.id == 1){
            runPoint(26,0,0);
            runPoint(26,-26,0);
            runPoint(39,-30,0);
        }
        else if(detectedTag.id == 3){
            runPoint(26,0,0);
            runPoint(26,26,0);
            runPoint(39,30,0);
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
    public void runPoint(double x, double y, double heading){
        desiredPose = new Pose2d(x,y,heading);
        localizer.update();
        pose = localizer.getPoseEstimate();
        distance = Math.abs(Math.hypot(desiredPose.getX()-pose.getX(),desiredPose.getY()-pose.getY()));
        Pose2d startPose = pose;
        if (lastX != x || lastY != y) {
            lastX = x;
            lastY = y;
            temp = new Pose2d(pose.getX(), pose.getY(),pose.getHeading());
            auto.driveToPoint(pose,desiredPose,temp,true);
        }
        else{ lastX = x; lastY = y; }
        auto.driveToPoint(pose,desiredPose,startPose,true);
    }
}

