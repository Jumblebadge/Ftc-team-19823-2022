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

import org.firstinspires.ftc.teamcode.subsystems.IMU;
import org.firstinspires.ftc.teamcode.utility.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.pipelines.cameraActivity;
import org.firstinspires.ftc.teamcode.navigation.GoToPoint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.openftc.apriltag.AprilTagDetection;

import java.util.List;


    @Config
    @Autonomous(name="parkAuto", group="Linear Opmode")
    public class parkAuto extends LinearOpMode {

        Pose2d pose;
        Pose2d desiredPose;
        GoToPoint auto;
        Localizer localizer;
        List<LynxModule> allHubs;
        IMU imu;

        public void runOpMode() {
            telemetry.addData("Status", "Initialized");
            cameraActivity webcamStuff = new cameraActivity(hardwareMap,telemetry);

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            webcamStuff.initCamera();
            telemetry.addLine("Waiting for start");

            //Initialize FTCDashboard telemetry
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            //Bulk sensor reads
            allHubs = hardwareMap.getAll(LynxModule.class);

            //Initialize FTCDashboard
            FtcDashboard dashboard = FtcDashboard.getInstance();

            //Create objects for the classes we use
            SwerveDrive drive = new SwerveDrive(telemetry, hardwareMap, true);
            auto = new GoToPoint(drive,telemetry,dashboard);
            //Bulk sensor reads
            for (LynxModule module : allHubs) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }

            //Fast loop go brrr
            PhotonCore.enable();
            localizer = new TwoWheelTrackingLocalizer(hardwareMap);

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
                runPoint(28,0,0);
                runPoint(28,24,0);
                runPoint(39,24,0);
            }
            else if(detectedTag.id == 3){
                runPoint(26,0,0);
                runPoint(28,-24,0);
                runPoint(39,-24,0);
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

