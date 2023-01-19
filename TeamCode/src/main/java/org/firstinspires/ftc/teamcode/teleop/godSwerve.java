package org.firstinspires.ftc.teamcode.teleop;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.*;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.lynx.*;
import com.acmerobotics.dashboard.*;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.subs.Toggler;
import org.firstinspires.ftc.teamcode.subs.driveSwerve;
import org.firstinspires.ftc.teamcode.subs.linearSlide;
import org.firstinspires.ftc.teamcode.subs.motorGroup;
import org.firstinspires.ftc.teamcode.subs.runMotionProfile;

import java.util.List;


@Config
@TeleOp(name="godSwerve", group="Linear Opmode")
public class godSwerve extends LinearOpMode {

    //Initialize FTCDashboard
    FtcDashboard dashboard;

    //Tuning values so that wheels are always facing straight (accounts for encoder drift - tuned manually)
    public static double mod3PC = -30, mod1PC = 10, mod2PC = -30;
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

        Servo inRotL = hardwareMap.get(Servo.class,"inL");
        Servo inRotR = hardwareMap.get(Servo.class,"inR");
        Servo outRotL = hardwareMap.get(Servo.class,"outL");
        Servo outRotR = hardwareMap.get(Servo.class,"outR");
        Servo claw = hardwareMap.get(Servo.class,"claw");
        Servo linkage = hardwareMap.get(Servo.class,"linkage");

        mod2m2.setDirection(DcMotorSimple.Direction.REVERSE);
        //mod3m2.setDirection(DcMotorSimple.Direction.REVERSE);
        //mod1m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mod1m2.setDirection(DcMotorSimple.Direction.REVERSE);

        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        //Initialize FTCDashboard
        dashboard = FtcDashboard.getInstance();

        //Create objects for the classes we use for swerve and PIDS

        driveSwerve drive = new driveSwerve(telemetry,mod1m1,mod1m2,mod2m1,mod2m2,mod3m1,mod3m2,mod1E,mod2E,mod3E,IMU,allHubs,vSensor, true);

        motorGroup slideMotors = new motorGroup(liftLeft,liftRight);

        linearSlide slide = new linearSlide(slideMotors);

        runMotionProfile linkageServoProfile = new runMotionProfile(1,1,1,0,0,0,0);
        runMotionProfile depositServosProfile = new runMotionProfile(1,1,1,0,0,0,0);
        runMotionProfile intakeServosProfile = new runMotionProfile(1,1,1,0,0,0,0);

        Toggler right_trigger = new Toggler();
        Toggler right_bumper = new Toggler();
        Toggler left_trigger = new Toggler();
        Toggler left_bumper = new Toggler();

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
            drive.setPIDCoeffs(Kp,Kd,Ki,Kf);

            drive.driveOut(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x/2);

            if (gamepad2.a) {
                liftTarget = 0;
            }
            else if (gamepad2.b) {
                liftTarget = 315;
            }
            else if (gamepad2.x) {
                liftTarget = 700;
            }
            else if (gamepad2.y) {
                liftTarget = 1100;
            }

            slide.moveTo(liftTarget);


            //linkage activated by rising edge detector
            if (right_trigger.update(gamepad2.right_trigger > 0.1)){
                if (linkage.getPosition() > 0.3){
                    //linkage.setPosition(linkageServoProfile.profiledServoMovement(0.15,linkage.getPosition()));
                    //linkage.setPosition(0.15);
                    //out
                }
                else if (linkage.getPosition() < 0.3){
                    //linkage.setPosition(0.5);
                    //in
                }
            }

            telemetry.addData("linkage",linkage.getPosition());

            //rising edge detector for claw open/close
            if(currentGamepad2.left_bumper && !previousGamepad2.left_bumper){
                if (claw.getPosition() < 0.45){
                    //claw.setPosition(0.6);
                    //open
                }
                else if (claw.getPosition() > 0.45){
                    //claw.setPosition(0.275);
                    //close
                }
            }

            //rising edge detector for outtake positions
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper){
                if (outRotL.getPosition()<0.5){
                   //outRotL.setPosition(1);
                   //outRotR.setPosition(1-outRotL.getPosition());
                   //in
                }
                else if (outRotL.getPosition()>0.5){
                    //outRotL.setPosition(0.1);
                    //outRotR.setPosition(1-outRotL.getPosition());
                    //out
                }
            }
            //right side is in 0 CHUB
            //left side is in 1 CHUB

            if(gamepad2.left_trigger>0.1){
                //inRotL.setPosition(0.5);
                //inRotR.setPosition(1-inRotL.getPosition());
                //straight up
            }
            else if (gamepad2.dpad_down){
                //inRotL.setPosition(0.865);
                //inRotR.setPosition(1-inRotL.getPosition());
                //down
            }
            else if (gamepad2.dpad_up){
                //inRotL.setPosition(0.3);
                //inRotR.setPosition(1-inRotL.getPosition());
                //up
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
