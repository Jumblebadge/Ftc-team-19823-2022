package org.firstinspires.ftc.teamcode.teleop;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.profile.*;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.lynx.*;
import com.acmerobotics.dashboard.*;
import com.qualcomm.robotcore.util.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.maths.*;
import org.firstinspires.ftc.teamcode.subs.drive;
import java.util.List;


@Config
@TeleOp(name="swervy", group="Linear Opmode")
public class swervy extends LinearOpMode {

    //Initialize FTCDashboard
    FtcDashboard dashboard;

    //Define reference variables for modules' heading
    double mod2reference=0,mod3reference=0;
    double mod1reference=0;

    //Timers for the PID loops
    ElapsedTime mod3timer =  new ElapsedTime(); ElapsedTime mod2timer =  new ElapsedTime(); ElapsedTime mod1timer =  new ElapsedTime();
    ElapsedTime hztimer = new ElapsedTime();
    //Define module position variables
    double mod1P = 0, mod2P = 0, mod3P = 0;

    //Define variables for power of wheels
    double mod1power = 0,mod2power = 0,mod3power = 0;

    //Tuning values so that wheels are always facing straight (accounts for encoder drift - tuned manually)
    public static double mod3PC = -120, mod1PC = -9, mod2PC = -55;

    //IMU
    BNO055IMU IMU;
    Localizer localizer;
    public static double Kp,Kd,Ki,headingTarget=0,parX=0,parY=0,perpX=0,perpY=0;
    ElapsedTime botRotTime = new ElapsedTime();


    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

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

        mod1m2.setDirection(DcMotorSimple.Direction.REVERSE);
        mod2m2.setDirection(DcMotorSimple.Direction.REVERSE);
        //mod3m2.setDirection(DcMotorSimple.Direction.REVERSE);

        //liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //liftRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        //Initialize FTCDashboard
        dashboard = FtcDashboard.getInstance();

        //Create objects for the classes we use for swerve and PIDS
        swerveMaths swavemath = new swerveMaths();

        controlLoopMath mod1PID = new controlLoopMath(0.1,0.0001,0.0009,0,mod1timer);
        controlLoopMath mod2PID = new controlLoopMath(0.1,0.0001,0.0009,0,mod2timer);
        controlLoopMath mod3PID = new controlLoopMath(0.1,0.0001,0.0009,0,mod3timer);
        localizer = new TwoWheelTrackingLocalizer(hardwareMap,IMU);

        drive drivein = new drive(telemetry,mod1m1,mod1m2,mod2m1,mod2m2,mod3m1,mod3m2,mod1E,mod2E,mod3E,IMU,mod1PID,mod2PID,mod3PID,swavemath,allHubs,null, true);

        //Bulk sensor reads
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //Fast loop go brrr
        PhotonCore.enable();

        waitForStart();
        while (opModeIsActive()) {

            controlLoopMath rotPID = new controlLoopMath(Kp,Kd,Ki,0,botRotTime);

            localizer.update();
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            Pose2d pose = localizer.getPoseEstimate();
            fieldOverlay.setStrokeWidth(1);
            fieldOverlay.setStroke("#3F51B5");
            drawRobot(fieldOverlay, pose);

            packet.put("x", pose.getX());
            packet.put("y", pose.getY());
            packet.put("heading (deg)", Math.toDegrees(pose.getHeading()));

            dashboard.sendTelemetryPacket(packet);
            double rotPIDout =0;
            if (Math.abs(rotPID.PIDout(mathsOperations.angleWrap(headingTarget-Math.toDegrees(pose.getHeading()))))>0.1) {
                rotPIDout = rotPID.PIDout(mathsOperations.angleWrap(headingTarget - Math.toDegrees(pose.getHeading())));
            }

            //telemetry.addData("x",pose.getX());
            //telemetry.addData("y",pose.getY());
            //telemetry.addData("heading",Math.toDegrees(pose.getHeading()));
            //telemetry.addData("gx",gamepad1.left_stick_x);
            //telemetry.addData("gy",gamepad1.left_stick_y);
            //telemetry.addData("headingout",rotPIDout);
            //telemetry.addData("headingtarget",headingTarget);
            headingTarget += gamepad1.right_stick_x;

            drivein.driveOut(gamepad1.left_stick_x,-gamepad1.left_stick_y,gamepad1.right_stick_x,mod1PC,mod2PC,mod3PC);

            //telemetry.addData("mod1reference",mod1reference);
            //telemetry.addData("mod2reference",mod2reference);
            //telemetry.addData("mod3reference",mod3reference);

            //telemetry.addData("mod1P",mod1P);
            //telemetry.addData("mod2P",mod2P);
            //telemetry.addData("mod3P",mod3P);

            //telemetry.addData("mod3power",mod3power);
            //telemetry.addData("mod2power",mod2power);
            //telemetry.addData("mod1power",mod1power);
            //telemetry.addData("bore1",mod3m1.getCurrentPosition());
            //telemetry.addData("bore2",mod3m2.getCurrentPosition());
            telemetry.setAutoClear(false);
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

