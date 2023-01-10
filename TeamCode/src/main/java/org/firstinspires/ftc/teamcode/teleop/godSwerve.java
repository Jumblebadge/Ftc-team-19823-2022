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
import com.qualcomm.robotcore.util.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.maths.*;
import org.firstinspires.ftc.teamcode.subs.drive;
import org.firstinspires.ftc.teamcode.subs.runServoMotionProfile;
import org.firstinspires.ftc.teamcode.subs.runMotorMotionProfile;

import java.util.List;


@Config
@TeleOp(name="godSwerve", group="Linear Opmode")
public class godSwerve extends LinearOpMode {

    //Initialize FTCDashboard
    FtcDashboard dashboard;

    //Timers for the PID loops
    ElapsedTime hztimer = new ElapsedTime();
    ElapsedTime RliftPROtime = new ElapsedTime(); ElapsedTime LliftPROtime = new ElapsedTime();

    //Tuning values so that wheels are always facing straight (accounts for encoder drift - tuned manually)
    public static double mod3PC = -120, mod1PC = -9, mod2PC = -55;
    public static double Kp=0.2,Kd=0.0001,Ki=0.0007,maxVel=1,maxAccel=1,maxJerk=1;

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
        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        //Initialize FTCDashboard
        dashboard = FtcDashboard.getInstance();

        //Create objects for the classes we use for swerve and PIDS

        controlLoopMath LliftPID = new controlLoopMath(0.2,0,0,0);
        controlLoopMath RliftPID = new controlLoopMath(0.2,0,0,0);

        drive drivein = new drive(telemetry,mod1m1,mod1m2,mod2m1,mod2m2,mod3m1,mod3m2,mod1E,mod2E,mod3E,IMU,allHubs,vSensor, true);

        MotionProfile LliftProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0,0,0),new MotionState(1,0,0),1,1,1);
        MotionProfile RliftProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0,0,0),new MotionState(1,0,0),1,1,1);

        runMotorMotionProfile test = new runMotorMotionProfile(liftLeft);
        test.setMotionConstraints(10000,11000,20000);

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

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        double LliftLastTarget = 0, RliftLastTarget = 0;

        waitForStart();
        while (opModeIsActive()) {
            drivein.setPIDCoeffs(Kp,Kd,Ki);

            try {
                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);

                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);
            }
            catch (RobotCoreException e) {
                //ah
            }

            hztimer.reset();

            drivein.driveOut(gamepad1.left_stick_x,-gamepad1.left_stick_y,-gamepad1.right_stick_x/2,gamepad1);

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
            LliftTarget = liftTarget;
            RliftTarget = -liftTarget;


            //linkage activated by rising edge detector
            boolean gp2RTC = (currentGamepad2.right_trigger > 0.1);
            boolean gp2RTB = (previousGamepad2.right_trigger > 0.1);

            if (gp2RTC && !gp2RTB) {
                if (linkage.getPosition() > 0.3){
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

            if (LliftLastTarget != LliftTarget) {
                LliftLastTarget = LliftTarget;
                LliftProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(liftLeft.getCurrentPosition(), 0, 0), new MotionState(LliftTarget, 0, 0), 10000, 11000,20000);
                LliftPROtime.reset();
            }
            else{ LliftLastTarget = LliftTarget; }
            MotionState LliftState = LliftProfile.get(LliftPROtime.seconds());
            liftLeft.setPower(LliftPID.PIDout(LliftState.getX()-liftLeft.getCurrentPosition()));


            if (RliftLastTarget != RliftTarget) {
                RliftLastTarget = RliftTarget;
                RliftProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(liftRight.getCurrentPosition(), 0, 0), new MotionState(RliftTarget, 0, 0), 10000, 11000,20000);
                RliftPROtime.reset();
            }
            else{ RliftLastTarget = RliftTarget; }

            MotionState RliftState = RliftProfile.get(RliftPROtime.seconds());
            liftRight.setPower(RliftPID.PIDout(RliftState.getX()-liftRight.getCurrentPosition()));

            telemetry.addLine(String.valueOf(1/hztimer.seconds()));
            telemetry.addData("LliftPos",liftLeft.getCurrentPosition());
            telemetry.addData("RliftPos",liftRight.getCurrentPosition());
            telemetry.addData("lliftstate",LliftState.getX());
            telemetry.addData("rliftstate",RliftState.getX());
            telemetry.addData("target",liftTarget);
            telemetry.update();
        }
    }
}
