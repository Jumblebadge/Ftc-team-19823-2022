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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.maths.slewRateLimiter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.twoServoBucket;
import org.firstinspires.ftc.teamcode.utility.Toggler;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.linearSlide;

import java.util.List;


@Config
@TeleOp(name="godSwerve", group="Linear Opmode")
public class godSwerve extends LinearOpMode {

    FtcDashboard dashboard;

    //Tuning values so that wheels are always facing straight (accounts for encoder drift - tuned manually)
    public static double mod3PC = 20, mod1PC = 10, mod2PC = -60;
    public static double Kp=0.2,Kd=0.0005,Ki=0.0007,Kf = 1;

    public static double depositTarget = 0.5, clawTarget = 0.5, linkageTarget = 0.5, intakeTarget = 0.5, slideTarget = 0, turretTarget = 0;

    //IMU
    BNO055IMU IMU;
    Orientation angles;

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

        mod2m2.setDirection(DcMotorSimple.Direction.REVERSE);
        //mod3m2.setDirection(DcMotorSimple.Direction.REVERSE);
        //mod1m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mod1m2.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx liftLeftMotor = hardwareMap.get(DcMotorEx.class, "Llift");
        DcMotorEx liftRightMotor = hardwareMap.get(DcMotorEx.class,"Rlift");

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

        liftLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        //Initialize FTCDashboard
        dashboard = FtcDashboard.getInstance();

        //class to drive the swerve
        SwerveDrive drive = new SwerveDrive(telemetry,mod1m1,mod1m2,mod2m1,mod2m2,mod3m1,mod3m2,mod1E,mod2E,mod3E,IMU, vSensor, true);

        //class that runs our linear slide
        linearSlide slide = new linearSlide(liftLeft,liftRight);
        slide.resetEncoders();

        twoServoBucket deposit = new twoServoBucket(depositRotationServoLeft,depositRotationServoRight);
        twoServoBucket intake = new twoServoBucket(inRotL,inRotR);
        Turret turret = new Turret(turretServo, turretPosition);

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

            //Clear the cache for better loop times (bulk sensor reads)
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            angles   = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = angles.firstAngle*-1;

            drive.setModuleAdjustments(mod1PC,mod2PC,mod3PC);

            drive.driveOut(leftX.rateLimit(gamepad1.left_stick_x,4),leftY.rateLimit(gamepad1.left_stick_y,4),rightX.rateLimit(gamepad1.right_stick_x*gamepad1.right_stick_x*gamepad1.right_stick_x,4));


            if (gamepad2.a) {
                slideTarget = 0;
            }
            else if (gamepad2.b) {
                slideTarget = 200;
            }
            else if (gamepad2.x) {
                slideTarget = 700;
            }
            else if (gamepad2.y) {
                slideTarget = 1100;
            }

            //rising edge detector for linkage out/in
            linkageTarget = (right_bumper.update(gamepad2.right_bumper) ? 0.7 : 0.3);

            //rising edge detector for claw open/close
            clawTarget = (left_bumper.update(gamepad2.left_bumper) ? 0.175 : 0.5);


            //rising edge detector for outtake positions
            depositTarget = (right_trigger.update(gamepad2.right_trigger > 0.1) ? 0.35 : 0.85);

            if(gamepad2.dpad_right){
                intakeTarget = 0.45;
                //straight up
            }
            else if (gamepad2.dpad_down){
                intakeTarget = 0.25;
                //down
            }
            else if (gamepad2.dpad_up){
                intakeTarget = 1;
                //up
            }

            turretTarget += (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x);
            turretTarget = Range.clip(turretTarget, -90,90);
            if (gamepad2.right_stick_button){
                turretTarget = 0;
            }
            turret.moveTo(turretTarget);
            linkage.setPosition(linkageTarget);
            claw.setPosition(clawTarget);
            deposit.moveTo(depositTarget);
            intake.moveTo(intakeTarget);
            slide.moveTo(-slideTarget);

            telemetry.addData("turrettarget",turretTarget);
            telemetry.addData("slidetarget",slideTarget);
            telemetry.addData("heading",heading);
            telemetry.update();
        }
    }
}
