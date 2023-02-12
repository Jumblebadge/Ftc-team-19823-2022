package org.firstinspires.ftc.teamcode.teleop;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.lynx.*;
import com.acmerobotics.dashboard.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.maths.PIDcontroller;
import org.firstinspires.ftc.teamcode.maths.mathsOperations;
import org.firstinspires.ftc.teamcode.maths.slewRateLimiter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.linearSlide;
import org.firstinspires.ftc.teamcode.subsystems.twoServoBucket;
import org.firstinspires.ftc.teamcode.utility.Toggler;
import org.firstinspires.ftc.teamcode.utility.myDcMotorEx;

import java.util.List;


@Config
@TeleOp(name="testing", group="Linear Opmode")
public class testing extends LinearOpMode {

    //Initialize FTCDashboard
    FtcDashboard dashboard;
    public static double depositTarget = 0.5, clawTarget = 0.5, linkageTarget = 0.5, intakeTarget = 0.5, slideTarget = 0, turretTarget = 0;
    public static double adjust = 0, Kp = 0, Kd = 0, Ki = 0, Kf = 0,r = 1000;
    public static boolean update = false;

    enum automation{
        IDLE,
        DEPOSIT,
        LINKAGE,
        TURRET,
        CLAW,
        EXTERNAL
    }

    automation state = automation.EXTERNAL;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Initialize FTCDashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

        //class that runs our linear slide
        linearSlide slide = new linearSlide(hardwareMap);
        slide.resetEncoders();

        twoServoBucket deposit = new twoServoBucket(depositRotationServoLeft,depositRotationServoRight);
        twoServoBucket intake = new twoServoBucket(inRotL,inRotR);
        Turret turret = new Turret(turretServo, turretPosition);

        Toggler right_trigger = new Toggler();
        Toggler right_bumper = new Toggler();
        Toggler left_bumper = new Toggler();

        ElapsedTime linkageTimer = new ElapsedTime();

        //Bulk sensor reads
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //Fast loop go brrr
        PhotonCore.enable();

        waitForStart();
        while (opModeIsActive()) {

            //Clear the cache for better loop times (bulk sensor reads)
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            switch(state){
                case EXTERNAL:
                    if (gamepad2.a) {
                        slideTarget = 0;
                    }
                    else if (gamepad2.b) {
                        slideTarget = 250;
                    }
                    else if (gamepad2.x) {
                        slideTarget = 700;
                    }
                    else if (gamepad2.y) {
                        slideTarget = 1100;
                    }

                    //rising edge detector for linkage out/in
                    linkageTarget = (left_bumper.update(gamepad2.left_bumper) ? 0.7 : 0.3);

                    //rising edge detector for claw open/close
                    clawTarget = (right_bumper.update(gamepad2.right_bumper) ? 0.175 : 0.5);


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

                    if (gamepad2.left_trigger > 2){
                        state = automation.LINKAGE;
                    }
                    break;

                case LINKAGE:
                    linkageTarget = 0.7;

                    if (gamepad2.left_trigger > 0.1){
                        state = automation.CLAW;
                    }
                    break;

                case CLAW:
                    clawTarget = 0.175;
                    turretTarget = 0;
                    linkageTarget = 0.3;

                    if (gamepad2.left_trigger > 0.1){
                        linkageTimer.reset();
                        state = automation.DEPOSIT;
                    }
                    break;

                case DEPOSIT:
                    if (linkageTimer.seconds() > 0.4){
                        clawTarget = 0.5;

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

                        if (slide.getError() < 25 && slideTarget != 0){
                            depositTarget = 0.35;
                            linkageTimer.reset();

                            if (linkageTimer.seconds() > 0.3){
                                depositTarget = 0.85;
                                slideTarget = 0;
                                state = automation.EXTERNAL;
                            }
                        }

                    }
                    break;
            }

            if (gamepad2.left_stick_button && state != automation.EXTERNAL){
                state = automation.EXTERNAL;
            }

            turretTarget += (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x);
            turretTarget = Range.clip(turretTarget, -90,90);
            if (gamepad2.right_stick_button){
                turretTarget = 0;
            }

            turret.setAdjust(adjust);
            turret.moveTo(turretTarget);
            linkage.setPosition(linkageTarget);
            claw.setPosition(clawTarget);
            deposit.moveTo(depositTarget);
            intake.moveTo(intakeTarget);
            slide.moveTo(-slideTarget);

            //x y a b is slide positions
            //lleft bumper toggled for open close claw
            //linkage toggle right bumper
            //deposit right trigger
            //dpad is claw rotation righ is middle down is in the robot, up is on the floor
            //linkage in: 0.3, linkage out: 0.7
            //bucket all the way down: linkage 0.5, intake 0.14
            //linkage all the way in: intake 0.25, slides 200

            telemetry.addData("turretpos",turret.getHeading());
            telemetry.addData("lLift",liftLeftMotor.getCurrentPosition());
            telemetry.addData("rLift",liftRightMotor.getCurrentPosition());
            telemetry.addData("lifttarget",slideTarget);
            telemetry.addData("turrettarget",turretTarget);
            telemetry.update();

        }
    }
}
