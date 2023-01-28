package org.firstinspires.ftc.teamcode.teleop;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.lynx.*;
import com.acmerobotics.dashboard.*;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.maths.slewRateLimiter;
import org.firstinspires.ftc.teamcode.subsystems.twoServoBucket;
import org.firstinspires.ftc.teamcode.utility.Toggler;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.linearSlide;
import org.firstinspires.ftc.teamcode.utility.runMotionProfile;

import java.util.List;


@Config
@TeleOp(name="godSwerve", group="Linear Opmode")
public class godSwerve extends LinearOpMode {

    //Initialize FTCDashboard
    FtcDashboard dashboard;

    //Tuning values so that wheels are always facing straight (accounts for encoder drift - tuned manually)
    public static double mod3PC = 20, mod1PC = 10, mod2PC = -60;
    public static double Kp=0.2,Kd=0.0005,Ki=0.0007,Kf = 1,maxVel=1,maxAccel=1,maxJerk=1;

    double RliftTarget = 1, LliftTarget = 1, liftTarget = 0;

    double linkagePos = 0.5;

    //IMU
    BNO055IMU IMU;

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

        DcMotorEx liftLeft = hardwareMap.get(DcMotorEx.class,"Llift");
        DcMotorEx liftRight = hardwareMap.get(DcMotorEx.class,"Rlift");

        VoltageSensor vSensor = hardwareMap.voltageSensor.iterator().next();

        //Servo inRotL = hardwareMap.get(Servo.class,"inL");
        //Servo inRotR = hardwareMap.get(Servo.class,"inR");
        //Servo outRotL = hardwareMap.get(Servo.class,"outL");
        //Servo outRotR = hardwareMap.get(Servo.class,"outR");
        //Servo claw = hardwareMap.get(Servo.class,"claw");
        //Servo linkage = hardwareMap.get(Servo.class,"linkage");

        mod2m2.setDirection(DcMotorSimple.Direction.REVERSE);
        //mod3m2.setDirection(DcMotorSimple.Direction.REVERSE);
        //mod1m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mod1m2.setDirection(DcMotorSimple.Direction.REVERSE);

        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        //Initialize FTCDashboard
        dashboard = FtcDashboard.getInstance();


        //class to drive the swerve
        SwerveDrive drive = new SwerveDrive(telemetry,mod1m1,mod1m2,mod2m1,mod2m2,mod3m1,mod3m2,mod1E,mod2E,mod3E,IMU,allHubs,vSensor, true);

        //class that runs our linear slide
        linearSlide slide = new linearSlide(liftLeft,liftRight);

        //twoServoBucket intake = new twoServoBucket(inRotL,inRotR);
        //twoServoBucket deposit = new twoServoBucket(outRotL,outRotR);

        Toggler right_trigger = new Toggler();
        Toggler right_bumper = new Toggler();
        Toggler left_bumper = new Toggler();

        slewRateLimiter leftX = new slewRateLimiter(), leftY = new slewRateLimiter(), rightX = new slewRateLimiter();

        drive.setModuleAdjustments(0,-15,-45);

        //Bulk sensor reads
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //Fast loop go brrr
        PhotonCore.enable();

        //linkage.setPosition(linkagePos);
        //claw.setPosition(0.6);
        //outRotL.setPosition(1);
        //outRotR.setPosition(1-outRotL.getPosition());
        //inRotL.setPosition(0.3);
        //inRotR.setPosition(1-inRotL.getPosition());

        waitForStart();
        while (opModeIsActive()) {

            drive.setModuleAdjustments(mod1PC,mod2PC,mod3PC);
            //drive.setPIDCoeffs(Kp,Kd,Ki,Kf);

            drive.driveOut(leftX.rateLimit(gamepad1.left_stick_x,4),leftY.rateLimit(gamepad1.left_stick_y,4),rightX.rateLimit(gamepad1.right_stick_x/2,4));

            //Clear the cache for better loop times (bulk sensor reads)
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            ///right bumper for the outtake
            ///right trigger for linkage
            ////left trigger for mid point of intake
            ///left bumper for claw open/close
            ////dpad up and down for up and down intake
            //letter buttons for slide positions
            //claw is on CHUB 2

            telemetry.addData("LliftPos",liftLeft.getCurrentPosition());
            telemetry.addData("RliftPos",liftRight.getCurrentPosition());
            telemetry.addData("target",liftTarget);
            telemetry.update();
        }
    }
}
