package org.firstinspires.ftc.teamcode.teleop;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.Angle;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.lynx.*;
import com.acmerobotics.dashboard.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.auto.cyclestuff;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.TwoServo;
import org.firstinspires.ftc.teamcode.utility.Toggler;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlide;

import java.util.List;


@Config
@TeleOp(name="godSwerve", group="Linear Opmode")
public class godSwerve extends LinearOpMode {

    //Tuning values so that wheels are always facing straight (accounts for encoder drift - tuned manually)
    public static double mod3PC = -20, mod1PC = -20, mod2PC = -105;
    public static double Kp = 0, Kd = 0, Ki = 0, Kf = 0, limit = 1000;

    double depositTarget = 0.3, clawTarget = 0.5, linkageTarget = 0.5, intakeTarget = 0.5, slideTarget = 0, turretTarget = 0, alignerTarget = 0.75;

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

        ServoImplEx depositRotationServoLeft = hardwareMap.get(ServoImplEx.class, "outL");
        ServoImplEx depositRotationServoRight = hardwareMap.get(ServoImplEx.class, "outR");
        ServoImplEx aligner = hardwareMap.get(ServoImplEx.class, "aligner");
        ServoImplEx inRotL = hardwareMap.get(ServoImplEx.class,"inL");
        ServoImplEx inRotR = hardwareMap.get(ServoImplEx.class,"inR");
        ServoImplEx linkage = hardwareMap.get(ServoImplEx.class, "linkage");
        ServoImplEx claw = hardwareMap.get(ServoImplEx.class, "claw");

        depositRotationServoLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
        depositRotationServoRight.setPwmRange(new PwmControl.PwmRange(500, 2500));
        aligner.setPwmRange(new PwmControl.PwmRange(500, 2500));
        inRotL.setPwmRange(new PwmControl.PwmRange(500,2500));
        inRotR.setPwmRange(new PwmControl.PwmRange(500,2500));
        linkage.setPwmRange(new PwmControl.PwmRange(500, 2500));
        claw.setPwmRange(new PwmControl.PwmRange(500,2500));


        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        //Initialize FTCDashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        //class to swerve the swerve
        SwerveDrive swerve = new SwerveDrive(telemetry, imu, hardwareMap, true);

        ElapsedTime hztimer = new ElapsedTime();

        //class that runs our linear slide
        LinearSlide slide = new LinearSlide(hardwareMap);
        slide.resetEncoders();

        TwoServo deposit = new TwoServo(depositRotationServoLeft,depositRotationServoRight);
        TwoServo intake = new TwoServo(inRotL,inRotR);
        Turret turret = new Turret(hardwareMap);

        Toggler right_trigger = new Toggler();
        Toggler right_bumper = new Toggler();
        Toggler left_bumper = new Toggler();

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

            swerve.drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y,gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x);

            if (gamepad2.a) {
                slide.zero(gamepad2.left_trigger > 0.1);
            }
            else if (gamepad2.b) {
                slide.transfer();
            }
            else if (gamepad2.x) {
                slide.mediumPole();
            }
            else if (gamepad2.y) {
                slide.highPole();
            }

            //rising edge detector for linkage out/in
            //alignerTarget = (left_trigger.update(gamepad2.left_trigger > 0.1) ? 1 : 0.75);

            linkageTarget = (right_bumper.update(gamepad2.right_bumper) ? 0.7 : 0.25);

            //rising edge detector for claw open/close
            clawTarget = (left_bumper.update(gamepad2.left_bumper) ? 0.2 : 0.5);

            //rising edge detector for outtake positions
            depositTarget = (right_trigger.update(gamepad2.right_trigger > 0.1) ? 0.8 : 0.3);

            if(gamepad2.dpad_right){
                intakeTarget = 0.45;
                //straight up
            }
            else if (gamepad2.dpad_down){
                intakeTarget = 0.3;
                //transfer
            }
            else if (gamepad2.dpad_up){
                intakeTarget = 0.975;
                //down
            }
            else if (gamepad2.dpad_left) {
                intakeTarget = 1;
            }

            turretTarget = 0;

            turret.moveTo(turretTarget);
            linkage.setPosition(linkageTarget);
            claw.setPosition(clawTarget);
            deposit.moveTo(depositTarget);
            intake.moveTo(intakeTarget);
            slide.update();
            swerve.rotateKids(AngleUnit.normalizeDegrees(cyclestuff.heading));
            swerve.setPIDCoeffs(Kp, Kd, Ki, Kf, limit);

            telemetry.addData("turrettarget",turretTarget);
            telemetry.addData("slidetarget",slideTarget);
            telemetry.addData("heading2", headingg);
            telemetry.addData("hz",1/hztimer.seconds());
            hztimer.reset();
            telemetry.update();
        }
    }
}
