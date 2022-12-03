package org.firstinspires.ftc.teamcode.teleop;

//Import EVERYTHING we need
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;import com.outoftheboxrobotics.photoncore.PhotonCore;import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.AnalogInput;import com.acmerobotics.dashboard.FtcDashboard;import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;import com.qualcomm.hardware.bosch.BNO055IMU;import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;import org.firstinspires.ftc.robotcore.external.navigation.Orientation;import org.firstinspires.ftc.teamcode.maths.controlLoopMath;import org.firstinspires.ftc.teamcode.maths.mathsOperations;import org.firstinspires.ftc.robotcore.external.navigation.Position;import org.firstinspires.ftc.teamcode.maths.swerveMaths;import java.util.List;import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.subs.drive;

import com.qualcomm.hardware.lynx.LynxModule;

@Config
@TeleOp(name="godSwerve", group="Linear Opmode")
public class godSwerve extends LinearOpMode {

    //Initialize FTCDashboard
    FtcDashboard dashboard;

    //Define reference variables for modules' heading
    double mod2reference=0,mod3reference=0;
    double mod1reference1 = 0, mod2reference1=0,mod3reference1=0;

    public static double mod1reference=0;

    //Timers for the PID loops
    ElapsedTime mod3timer =  new ElapsedTime(); ElapsedTime mod2timer =  new ElapsedTime(); ElapsedTime mod1timer =  new ElapsedTime();
    ElapsedTime hztimer = new ElapsedTime();
    ElapsedTime RliftPROtime = new ElapsedTime(); ElapsedTime LliftPROtime = new ElapsedTime(); ElapsedTime LliftPIDtime = new ElapsedTime(); ElapsedTime RliftPIDtime = new ElapsedTime();

    //Define module position variables
    double mod1P = 0, mod2P = 0, mod3P = 0;

    //Define variables for power of wheels
    double mod1power = 0,mod2power = 0,mod3power = 0;

    //Tuning values so that wheels are always facing straight (accounts for encoder drift - tuned manually)
    public static double mod3PC = -120, mod1PC = -9, mod2PC = -55;

    double RliftTarget = 1, LliftTarget = 1, inRotTarget = 0.5, liftTarget = 0, outRotTarget  = 1;

    public static double Kp = 0, Kd = 0, Ki = 0, Kf = 0;

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

        Servo inRotL = hardwareMap.get(Servo.class,"inRotL");
        Servo inRotR = hardwareMap.get(Servo.class,"inRotR");
        Servo outRotL = hardwareMap.get(Servo.class,"outRotL");
        Servo outRotR = hardwareMap.get(Servo.class,"outRotR");
        Servo clawI = hardwareMap.get(Servo.class,"clawI");
        Servo clawO = hardwareMap.get(Servo.class,"clawO");

        mod2m2.setDirection(DcMotorSimple.Direction.REVERSE);
        mod3m2.setDirection(DcMotorSimple.Direction.REVERSE);
        //mod1m2.setDirection(DcMotorSimple.Direction.REVERSE);

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
        controlLoopMath LliftPID = new controlLoopMath(0.2,0,0,0,LliftPIDtime);
        controlLoopMath RliftPID = new controlLoopMath(0.2,0,0,0,RliftPIDtime);

        drive drivein = new drive(telemetry,mod1m1,mod1m2,mod2m1,mod2m2,mod3m1,mod3m2,mod1E,mod2E,mod3E,IMU,mod1PID,mod2PID,mod3PID,swavemath,allHubs,null, true);

        //Bulk sensor reads
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //Fast loop go brrr
        PhotonCore.enable();

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        //Wraparound detection variables
        double LliftLastTarget = 0, RliftLastTarget = 0;
        clawI.setPosition(0.4);

        MotionProfile LliftProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0,0,0),new MotionState(1,0,0),1,1);
        MotionProfile RliftProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0,0,0),new MotionState(1,0,0),1,1);

        waitForStart();
        while (opModeIsActive()) {

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

            drivein.driveOut(-gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x);

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
                liftTarget = 1200;
            }
            LliftTarget = liftTarget;
            RliftTarget = -liftTarget;

            if (gamepad2.dpad_up) {
                inRotTarget = 0.62;
            }
            else if (gamepad2.dpad_down) {
                inRotTarget = 0.075;
            }
            else if (gamepad2.dpad_left) {
                inRotTarget = 0.5;
            }

            //down = 0.05
            //0.65 = swap
            //0.5 = rest


            if (gamepad2.right_trigger>0)  {
                outRotTarget = 0.15;
            }
            //flip out

            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                if (clawO.getPosition() == 0.2){
                    clawO.setPosition(0);
                    outRotTarget = 0.86;
                    //open and come in
                }
                else if (clawO.getPosition() == 0){
                    clawO.setPosition(0.2);
                    clawI.setPosition(0.9);
                    //close
                }
            }

            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                /*
                if (clawI.getPosition() == 0.9){
                    clawI.setPosition(0.4);
                    //close
                }
                else if (clawI.getPosition() == 0.4){
                    clawI.setPosition(0.9);
                    //open
                }

                 */
            }
            if (gamepad2.left_bumper) {
                clawI.setPosition(0.4);
            }
            else if (gamepad2.left_trigger>0){
                clawI.setPosition(0.9);
            }

            inRotL.setPosition(inRotTarget);
            inRotR.setPosition(1-inRotTarget);
            outRotL.setPosition(outRotTarget);
            outRotR.setPosition(1-outRotTarget);

            if (LliftLastTarget != LliftTarget) {
                LliftLastTarget = LliftTarget;
                LliftProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(liftLeft.getCurrentPosition(), 0, 0), new MotionState(LliftTarget, 0, 0), 7000, 5000,50000);
                LliftPROtime.reset();
            }
            else{ LliftLastTarget = LliftTarget; }

            MotionState LliftState = LliftProfile.get(LliftPROtime.seconds());
            liftLeft.setPower(LliftPID.PIDout(LliftState.getX()-liftLeft.getCurrentPosition()));


            if (RliftLastTarget != RliftTarget) {
                RliftLastTarget = RliftTarget;
                RliftProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(liftRight.getCurrentPosition(), 0, 0), new MotionState(RliftTarget, 0, 0), 7000, 5000,50000);
                RliftPROtime.reset();
            }
            else{ RliftLastTarget = RliftTarget; }

            MotionState RliftState = RliftProfile.get(RliftPROtime.seconds());
            liftRight.setPower(RliftPID.PIDout(RliftState.getX()-liftRight.getCurrentPosition()));

            telemetry.addData("mod1reference",mod1reference);
            telemetry.addData("mod2reference",mod2reference);
            telemetry.addData("mod3reference",mod3reference);

            telemetry.addData("mod1P",mod1P);
            telemetry.addData("mod2P",mod2P);
            telemetry.addData("mod3P",mod3P);

            telemetry.addData("mod3power",mod3power);
            telemetry.addData("mod2power",mod2power);
            telemetry.addData("mod1power",mod1power);

            telemetry.addLine(String.valueOf(1/hztimer.seconds()));
            telemetry.update();
        }
    }
}
