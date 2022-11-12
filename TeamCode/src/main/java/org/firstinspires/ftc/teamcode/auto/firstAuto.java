package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.maths.controlLoopMath;
import org.firstinspires.ftc.teamcode.maths.mathsOperations;
import org.firstinspires.ftc.teamcode.maths.swerveMaths;
import org.firstinspires.ftc.teamcode.subs.drive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.teamcode.pipelines.greyPipe;
import org.firstinspires.ftc.teamcode.pipelines.aprilTagDetectPipe;

import java.util.ArrayList;
import java.util.List;


@Config
@Autonomous(name="firstAuto", group="Linear Opmode")
public class firstAuto extends LinearOpMode {

    OpenCvWebcam webcam;

    int side1 = 1;
    int side2 = 2;
    int side3 = 3;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;

    aprilTagDetectPipe pipeline = new aprilTagDetectPipe(tagsize,fx,fy,cx,cy);

    AprilTagDetection detectedTag = null;


    //Initialize FTCDashboard
    FtcDashboard dashboard;

    //Timers for the PID loops
    ElapsedTime mod3timer =  new ElapsedTime(); ElapsedTime mod2timer =  new ElapsedTime(); ElapsedTime mod1timer =  new ElapsedTime();

    ElapsedTime autotime = new ElapsedTime();

    //IMU
    BNO055IMU IMU;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        telemetry.addLine("Waiting for start");
        FtcDashboard.getInstance().startCameraStream(webcam, 60);

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

        mod3m2.setDirection(DcMotorSimple.Direction.REVERSE);
        mod2m2.setDirection(DcMotorSimple.Direction.REVERSE);

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        //Initialize FTCDashboard
        dashboard = FtcDashboard.getInstance();

        //Create objects for the classes we use for swerve and PIDS
        swerveMaths swavemath = new swerveMaths();

        controlLoopMath mod1PID = new controlLoopMath(0.1,0.0001,0.0007,0,mod1timer);
        controlLoopMath mod2PID = new controlLoopMath(0.1,0.0001,0.0007,0,mod2timer);
        controlLoopMath mod3PID = new controlLoopMath(0.1,0.0001,0.0007,0,mod3timer);

        drive drivein = new drive(telemetry,mod1m1,mod1m2,mod2m1,mod2m2,mod3m1,mod3m2,mod1E,mod2E,mod3E,IMU,mod1PID,mod2PID,mod3PID,swavemath,allHubs);

        //Bulk sensor reads
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //Fast loop go brrr
        PhotonCore.enable();

        while (!isStarted() && !isStopRequested()) {

            ArrayList<AprilTagDetection> detecteds = pipeline.getLatestDetections();

            if(detecteds.size() != 0) {
                boolean tagFound = false;

                for(AprilTagDetection tag : detecteds) {
                    if(tag.id == side1 || tag.id == side2 || tag.id == side3) {
                        detectedTag = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound) {
                    telemetry.addLine("can see tag \n");
                    tagToTelemetry(detectedTag);
                }
                else
                {
                    telemetry.addLine("cant see tag");

                    if(detectedTag == null)
                    {
                        telemetry.addLine("never seen tag");
                    }
                    else
                    {
                        telemetry.addLine("\nlast seen tag");
                        tagToTelemetry(detectedTag);
                    }
                }

            }
            else
            {
                telemetry.addLine("cant see tag");

                if(detectedTag == null)
                {
                    telemetry.addLine("neverr seen tag");
                }
                else
                {
                    telemetry.addLine("\nlast seen tag");
                    tagToTelemetry(detectedTag);
                }

            }

            telemetry.update();
            sleep(20);
        }


        webcam.closeCameraDevice();
        //auto starting

        if(detectedTag == null || detectedTag.id == side2) {
            autotime.reset();
            while (autotime.seconds()<2.5&&opModeIsActive()){
                drivein.driveOut(0,-0.3,0);
            }
            while(autotime.seconds()<5&&opModeIsActive()){
                drivein.driveOut(0.01,0.01,0);
            }
        }

        else if(detectedTag.id == side1){
            autotime.reset();
            while(autotime.seconds()<2.7&&opModeIsActive()){
                drivein.driveOut(0,-0.3,0);
            }
            while (autotime.seconds()<4.5&&autotime.seconds()>2.7&&opModeIsActive()){
                drivein.driveOut(0.3,0,0);
            }
            while(autotime.seconds()<6&&opModeIsActive()){
                drivein.driveOut(0.01,0.01,0);
            }
        }
        else if(detectedTag.id == side3){
            autotime.reset();
            while(autotime.seconds()<2.5&&opModeIsActive()){
                drivein.driveOut(0,-0.3,0);
            }
            while (autotime.seconds()<4.5&&autotime.seconds()>2.5&&opModeIsActive()){
                drivein.driveOut(-0.3,0,0);
            }
            while(autotime.seconds()<6&&opModeIsActive()){
                drivein.driveOut(0.01,0.01,0);
            }

        }

        if(detectedTag != null)
        {
            telemetry.addLine("tag:\n");
            tagToTelemetry(detectedTag);
        }
        else
        {
            telemetry.addLine("never seen tag");
        }
        telemetry.update();

    }
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}

