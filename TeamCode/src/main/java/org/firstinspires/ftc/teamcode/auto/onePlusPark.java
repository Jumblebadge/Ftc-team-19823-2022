package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.pipelines.cameraActivity;

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
@Disabled
@Autonomous(name="onePlusPark", group="Linear Opmode")
public class onePlusPark extends LinearOpMode {

    OpenCvWebcam webcam;

    //Initialize FTCDashboard
    FtcDashboard dashboard;

    //Timers for the PID loops
    ElapsedTime mod3timer =  new ElapsedTime(); ElapsedTime mod2timer =  new ElapsedTime(); ElapsedTime mod1timer =  new ElapsedTime();

    ElapsedTime xPIDTime = new ElapsedTime(); ElapsedTime yPIDTime = new ElapsedTime(); ElapsedTime headingPIDTime = new ElapsedTime();

    //IMU
    BNO055IMU IMU;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        cameraActivity webcamStuff = new cameraActivity(hardwareMap,telemetry);

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
        IMU = hardwareMap.get(BNO055IMU.class, "imu");
        IMU.initialize(parameters);
        IMU.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //Initialize FTCDashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        //Initialize FTCDashboard
        dashboard = FtcDashboard.getInstance();

        //Create objects for the classes we use for swerve and PIDS
        SwerveDrive drivein = new SwerveDrive(telemetry, hardwareMap, true);

        //Bulk sensor reads
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //Fast loop go brrr
        PhotonCore.enable();

        while (!isStarted() && !isStopRequested()) {
            webcamStuff.detectTags();
            sleep(20);
        }
        webcam.closeCameraDevice();
        AprilTagDetection detectedTag = webcamStuff.sideDetected();
        //auto starting

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
}

