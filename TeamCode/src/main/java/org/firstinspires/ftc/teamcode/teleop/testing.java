package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.subsystems.TwoServo;
import org.firstinspires.ftc.teamcode.utility.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.pipelines.cameraActivity;
import org.firstinspires.ftc.teamcode.navigation.GoToPoint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;


@Config
@TeleOp(name="testing", group="Linear Opmode")
public class testing extends LinearOpMode {

    OpenCvWebcam webcam;

    Localizer localizer;

    //Initialize FTCDashboard
    FtcDashboard dashboard;

    double lastX = 0.0001, lastY = 0.0001;

    Pose2d pose = new Pose2d(0,0,0);
    Pose2d temp = new Pose2d(0,0,0);
    double cyclesCompleted = 0;

    enum apexStates {
        DRIVE_TO_CYCLE,
        CYCLING,
        PARK
    }

    enum cycleStates {
        INTAKE_GRAB,
        INTAKE_UP,
        DEPOSIT_EXTEND,
        DEPOSIT_DUMP,
        WAIT,
        TRANSFER
    }

    apexStates apexstate = apexStates.DRIVE_TO_CYCLE;
    cycleStates cyclestate = cycleStates.WAIT;

    GoToPoint auto;

    ElapsedTime goofytimer = new ElapsedTime();
    ElapsedTime autogoofytimer = new ElapsedTime();

    //TODO auto cycling position = new Pose2d(47, 10, -1)

    //IMU
    BNO055IMU imu;

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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        AnalogInput turretPosition = hardwareMap.get(AnalogInput.class, "turretMa3");

        ServoImplEx depositRotationServoLeft = hardwareMap.get(ServoImplEx.class, "outL");
        ServoImplEx depositRotationServoRight = hardwareMap.get(ServoImplEx.class, "outR");
        ServoImplEx inRotL = hardwareMap.get(ServoImplEx.class,"inL");
        ServoImplEx inRotR = hardwareMap.get(ServoImplEx.class,"inR");
        ServoImplEx linkage = hardwareMap.get(ServoImplEx.class, "linkage");
        CRServoImplEx turretServo = hardwareMap.get(CRServoImplEx.class, "turret");
        ServoImplEx claw = hardwareMap.get(ServoImplEx.class, "claw");

        depositRotationServoLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
        depositRotationServoRight.setPwmRange(new PwmControl.PwmRange(500, 2500));
        inRotL.setPwmRange(new PwmControl.PwmRange(500,2500));
        inRotR.setPwmRange(new PwmControl.PwmRange(500,2500));
        linkage.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turretServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        claw.setPwmRange(new PwmControl.PwmRange(500,2500));

        LinearSlide slide = new LinearSlide(hardwareMap);
        slide.resetEncoders();

        TwoServo deposit = new TwoServo(depositRotationServoLeft,depositRotationServoRight);
        TwoServo intake = new TwoServo(inRotL,inRotR);
        Turret turret = new Turret(turretServo, turretPosition);

        //Initialize FTCDashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        localizer = new TwoWheelTrackingLocalizer(hardwareMap,imu);

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        //Initialize FTCDashboard
        dashboard = FtcDashboard.getInstance();

        //Create objects for the classes we use
        SwerveDrive drive = new SwerveDrive(telemetry, imu, hardwareMap, true);
        auto = new GoToPoint(drive,telemetry,dashboard);

        //Bulk sensor reads
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //Fast loop go brrr
        PhotonCore.enable();

        while (!isStarted() && !isStopRequested()) {
            webcamStuff.detectTags();
            intake.moveTo(0.45);
            claw.setPosition(0.5);
            sleep(20);
        }
        webcamStuff.closeCamera();
        AprilTagDetection detectedTag = webcamStuff.sideDetected();
        apexstate = apexStates.DRIVE_TO_CYCLE;

        while (opModeIsActive()) {
            //Clear the cache for better loop times (bulk sensor reads)
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            localizer.update();
            pose = localizer.getPoseEstimate();

            runPoint(new Pose2d(0,0,0));

            switch (apexstate){
                case DRIVE_TO_CYCLE:
                    //drive to cycling position
                    //telemetry.addData("state","DRIVETOCYCLE");
                    apexstate = apexStates.CYCLING;
                    cyclestate = cycleStates.DEPOSIT_EXTEND;
                    break;

                case CYCLING:
                    if (cyclesCompleted == 6){
                        cyclestate = cycleStates.WAIT;
                        apexstate = apexStates.PARK;
                    }
                    break;

                case PARK:
                    //drive to park position
                    //telemetry.addData("state","PARK");
                    if(detectedTag == null || detectedTag.id == 2) {
                        //desiredPose = new Pose2d(39,0,0);
                        //desiredPose = new Pose2d(39,6,0);
                    }
                    else if(detectedTag.id == 1){
                        //desiredPose = new Pose2d(26,0,0);
                        //desiredPose = new Pose2d(26,-26,0);
                        //desiredPose = new Pose2d(39,-30,0);
                    }
                    else if(detectedTag.id == 3){
                        //desiredPose = new Pose2d(26,0,0);
                        //desiredPose = new Pose2d(26,26,0);
                        //desiredPose = new Pose2d(39,30,0);
                    }
                    break;
            }

            switch (cyclestate){
                case WAIT:
                    slide.zero();
                    deposit.moveTo(0.3);
                    break;

                case INTAKE_GRAB:
                    //intake extends and then retracts with cone, then drops into deposit box
                    slide.transfer();
                    deposit.moveTo(0.3);
                    intake.moveTo(1);
                    goofytimer.reset();
                    autogoofytimer.reset();
                    cyclestate = cycleStates.INTAKE_UP;
                    break;

                case INTAKE_UP:
                    if (autogoofytimer.seconds() > 0.5){
                        claw.setPosition(0.2);
                    }
                    if (goofytimer.seconds() > 1) {

                        intake.moveTo(0.3);
                        cyclestate = cycleStates.TRANSFER;
                        goofytimer.reset();
                        autogoofytimer.reset();
                    }
                    break;

                case TRANSFER:
                    if (autogoofytimer.seconds() > 0.5) {
                        claw.setPosition(0.5);
                    }
                    if (goofytimer.seconds() > 1) {

                        intake.moveTo(0.45);
                        cyclestate = cycleStates.DEPOSIT_EXTEND;
                    }
                    break;

                case DEPOSIT_EXTEND:
                    //lift slides, drop cone, come back down

                    slide.highPole();
                    if (slide.isPositionDone()) {
                        cyclestate = cycleStates.DEPOSIT_DUMP;
                        goofytimer.reset();
                        deposit.moveTo(0.8);
                    }
                    break;

                case DEPOSIT_DUMP:
                    if (goofytimer.seconds() > 0.75){
                        deposit.moveTo(0.3);
                        cyclesCompleted += 1;
                        cyclestate = cycleStates.INTAKE_GRAB;
                    }
                    break;
            }


            slide.update();
            turret.moveTo(0);
            linkage.setPosition(0.25);
            telemetry.addData("apexstate",apexstate.toString());
            telemetry.addData("cyclestate",cyclestate.toString());
            telemetry.addData("slide", slide.getTarget());
            telemetry.addData("X",pose.getX());
            telemetry.addData("Y",pose.getY());
            telemetry.addData("heading",pose.getHeading());
            telemetry.update();


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

    }

    public void runPoint(Pose2d desiredPose){
        if (lastX != desiredPose.getX() || lastY != desiredPose.getY()) {
            lastX = desiredPose.getX();
            lastY = desiredPose.getY();
            temp = new Pose2d(pose.getX(), pose.getY(),pose.getHeading());
            auto.driveToPoint(pose,desiredPose,temp,true);
        }
        else{ lastX = desiredPose.getX(); lastY = desiredPose.getY(); }
        auto.driveToPoint(pose,desiredPose,temp,false);
    }


}

