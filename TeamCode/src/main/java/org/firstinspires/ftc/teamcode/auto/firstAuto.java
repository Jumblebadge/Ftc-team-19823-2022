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

    //Define reference variables for modules' heading
    public static double mod1reference=0,mod2reference=0,mod3reference=0;
    public static double mod1reference1=0,mod2reference1=0,mod3reference1=0;

    //Timers for the PID loops
    ElapsedTime mod3timer =  new ElapsedTime(); ElapsedTime mod2timer =  new ElapsedTime(); ElapsedTime mod1timer =  new ElapsedTime();

    //Define module position variables
    double mod1P = 0, mod2P = 0, mod3P = 0;

    //Define variables for power of wheels
    double mod1power = 0,mod2power = 0,mod3power = 0;

    //Tuning values so that wheels are always facing straight (accounts for encoder drift - tuned manually)
    public static double mod3PC = -92, mod1PC = 0, mod2PC = -10;

    //IMU
    BNO055IMU IMU;
    Orientation angles;


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
                webcam.startStreaming(848, 480, OpenCvCameraRotation.UPRIGHT);
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
        //mod2m2.setDirection(DcMotorSimple.Direction.REVERSE);

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        //Initialize FTCDashboard
        dashboard = FtcDashboard.getInstance();

        //Create objects for the classes we use for swerve and PIDS
        swerveMaths swavemath = new swerveMaths();

        controlLoopMath mod1PID = new controlLoopMath(0.1,0.0001,0.0007,0,mod1timer);
        controlLoopMath mod2PID = new controlLoopMath(0.1,0.0001,0.0007,0,mod2timer);
        controlLoopMath mod3PID = new controlLoopMath(0.1,0.0001,0.0007,0,mod3timer);

        //Bulk sensor reads
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //Fast loop go brrr
        PhotonCore.enable();

        //Wraparound detection variables
        boolean mod1wrapped = false, mod2wrapped = false, mod3wrapped = false;
        double mod1lastpos = 0, mod2lastpos = 0, mod3lastpos = 0;


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
                    telemetry.addLine("never seen tag");
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

        while (opModeIsActive()) {


            //Clear the cache for better loop times (bulk sensor reads)
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            //Turn our MA3 absolute encoder signals from volts to degrees
            double mod1P1 = mod1E.getVoltage() * 74.16;
            double mod2P1 = mod2E.getVoltage() * 74.16;
            double mod3P1 = mod3E.getVoltage() * 74.16;

            //detecting wraparounds on the ma3's so that the 1:2 gear ratio does not matter
            //mod1P = mathsOperations.modWrap(mod1P1,mod1wrapped,mod1lastpos,2);
            //mod1lastpos = mod1P1;
            double mod1positiondelta = mod1P1 - mod1lastpos;
            mod1lastpos = mod1P1;

            mod1wrapped = ((mod1positiondelta > 180) != mod1wrapped);
            mod1wrapped = ((mod1positiondelta <-180) != mod1wrapped);
            mod1P = (mod1wrapped == true ? 180 + mod1P1/2 : mod1P1/2);

            //mod2P = mathsOperations.modWrap(mod2P1,mod2wrapped,mod2lastpos,2);
            //mod2lastpos = mod2P1;
            double mod2positiondelta = mod2P1 - mod2lastpos;
            mod2lastpos = mod2P1;

            mod2wrapped = ((mod2positiondelta > 180) != mod2wrapped);
            mod2wrapped = ((mod2positiondelta <-180) != mod2wrapped);
            mod2P = (mod2wrapped == true ? 180 + mod2P1/2 : mod2P1/2);

            //mod3P = mathsOperations.modWrap(mod3P1,mod3wrapped,mod3lastpos,2);
            //mod3lastpos = mod3P1;
            double mod3positiondelta = mod3P1 - mod3lastpos;
            mod3lastpos = mod3P1;

            mod3wrapped = ((mod3positiondelta > 180) != mod3wrapped);
            mod3wrapped = ((mod3positiondelta <-180) != mod3wrapped);
            mod3P = (mod3wrapped == true ? 180 + mod3P1/2 : mod3P1/2);

            telemetry.addData("mod1P",mod1P);
            telemetry.addData("mod2P",mod2P);
            telemetry.addData("mod3P",mod3P);

            //Update heading of robot
            angles   = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = angles.firstAngle*-1;

            telemetry.addData("IMU",heading);

            //Retrieve the angles and powers for all of our wheels from the swerve kinematics
            double[] output = swavemath.Math(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x,heading,true);
            mod1power=output[0];
            mod3power=output[2];
            mod2power=output[1];

            if (gamepad1.left_stick_y!=0||gamepad1.left_stick_x!=0||gamepad1.right_stick_x!=0){
                mod1reference1=output[3];
                mod3reference1=output[5];
                mod2reference1=output[4];
            }

            mod1reference=mod1reference1;
            mod3reference=mod3reference1;
            mod2reference=mod2reference1;

            if(detectedTag == null || detectedTag.id == side1) {
                //go to default position after auton code
            }
            else if(detectedTag.id == side2){

            }
            else if(detectedTag.id == side3){

            }

            //Subtract our tuning values to account for any encoder drift
            mod3P -= mod3PC;
            mod2P -= mod2PC;
            mod1P -= mod1PC;

            //Anglewrap all the angles so that the module turns both ways
            mod1P = mathsOperations.angleWrap(mod1P);
            mod2P = mathsOperations.angleWrap(mod2P);
            mod3P = mathsOperations.angleWrap(mod3P);

            mod1reference = mathsOperations.angleWrap(mod1reference);
            mod2reference = mathsOperations.angleWrap(mod2reference);
            mod3reference = mathsOperations.angleWrap(mod3reference);

            //Make sure that a module never turns more than 90 degrees
            double[] mod1efvalues = mathsOperations.efficientTurn(mod1reference,mod1P,mod1power);
            mod1reference=mod1efvalues[0];
            mod1power=mod1efvalues[1];

            double[] mod2efvalues = mathsOperations.efficientTurn(mod2reference,mod2P,mod2power);
            mod2reference=mod2efvalues[0];
            mod2power=mod2efvalues[1];

            double[] mod3efvalues = mathsOperations.efficientTurn(mod3reference,mod3P,mod3power);
            mod3reference=mod3efvalues[0];
            mod3power=mod3efvalues[1];

            //change coax values into diffy values, from pid and power
            double[] mod1values = mathsOperations.diffyConvert(mod1PID.PIDout(AngleUnit.normalizeDegrees(mod1reference-mod1P)),mod1power);
            mod1m1.setPower(mod1values[0]);
            mod1m2.setPower(mod1efvalues[1]);
            double[] mod2values = mathsOperations.diffyConvert(mod2PID.PIDout(AngleUnit.normalizeDegrees(mod2reference-mod2P)),mod2power);
            mod2m1.setPower(mod2values[0]);
            mod2m2.setPower(mod2values[1]);
            double[] mod3values = mathsOperations.diffyConvert(mod3PID.PIDout(AngleUnit.normalizeDegrees(mod3reference-mod3P)),mod3power);
            mod3m1.setPower(mod3values[0]);
            mod3m2.setPower(mod3values[1]);

            telemetry.update();

        }
    }
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}

