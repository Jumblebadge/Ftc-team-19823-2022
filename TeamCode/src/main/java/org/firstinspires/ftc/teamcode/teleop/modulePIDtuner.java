package org.firstinspires.ftc.teamcode.teleop;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
@TeleOp(name="modPIDtune", group="Linear Opmode")
public class modulePIDtuner extends LinearOpMode {

    //Tuning values so that wheels are always facing straight (accounts for encoder drift - tuned manually)
    public static double mod3PC = -20, mod1PC = -20, mod2PC = -105;
    public static double Kp = 0, Kd = 0, Ki = 0, Kf = 0;

    double depositTarget = 0.5, clawTarget = 0.5, linkageTarget = 0.5, intakeTarget = 0.5, slideTarget = 0, turretTarget = 0;

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
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //Initialize FTCDashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        /*
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
        */

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        //Initialize FTCDashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        //class to swerve the swerve
        SwerveDrive swerve = new SwerveDrive(telemetry, imu, hardwareMap, true);

        //class that runs our linear slide
        linearSlide slide = new linearSlide(hardwareMap);
        slide.resetEncoders();

        //twoServoBucket deposit = new twoServoBucket(depositRotationServoLeft,depositRotationServoRight);
        //twoServoBucket intake = new twoServoBucket(inRotL,inRotR);
        //Turret turret = new Turret(turretServo, turretPosition);

        Toggler right_trigger = new Toggler();
        Toggler right_bumper = new Toggler();
        Toggler left_bumper = new Toggler();

        swerve.setModuleAdjustments(0,-15,-45);

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

            Orientation angeles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double headingg = angeles.firstAngle*-1;

            swerve.setModuleAdjustments(mod1PC,mod2PC,mod3PC);
            swerve.setPIDCoeffs(Kp, Kd, Ki, Kf);

            swerve.drive(gamepad1.left_stick_x, gamepad1.left_stick_y,gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x);

            if (gamepad2.a) {
                slideTarget = 0;
            }
            else if (gamepad2.b) {
                slideTarget = 250;
            }
            else if (gamepad2.x) {
                slideTarget = 600;
            }
            else if (gamepad2.y) {
                slideTarget = 1050;
            }

            //rising edge detector for linkage out/in
            linkageTarget = (right_bumper.update(gamepad2.right_bumper) ? 0.7 : 0.25);

            //rising edge detector for claw open/close
            clawTarget = (left_bumper.update(gamepad2.left_bumper) ? 0.2 : 0.5);

            //rising edge detector for outtake positions
            depositTarget = (right_trigger.update(gamepad2.right_trigger > 0.1) ? 0.3 : 0.85);

            if(gamepad2.dpad_right){
                intakeTarget = 0.45;
                //straight up
            }
            else if (gamepad2.dpad_down){
                intakeTarget = 0.3;
                //down
            }
            else if (gamepad2.dpad_up){
                intakeTarget = 1;
                //up
            }

            turretTarget = 0;

            //turret.moveTo(turretTarget);
            //linkage.setPosition(linkageTarget);
            //claw.setPosition(clawTarget);
            //deposit.moveTo(depositTarget);
            //intake.moveTo(intakeTarget);
            //slide.moveTo(slideTarget);

            telemetry.addData("turrettarget",turretTarget);
            telemetry.addData("slidetarget",slideTarget);
            telemetry.addData("heading2", headingg);
            telemetry.update();
        }
    }
}
