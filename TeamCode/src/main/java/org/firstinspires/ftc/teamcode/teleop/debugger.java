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
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.TwoServo;
import org.firstinspires.ftc.teamcode.utility.Toggler;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlide;

import java.util.List;


@Config
@TeleOp(name="debugger", group="Linear Opmode")
public class debugger extends LinearOpMode {

    public static boolean activateLinkage = false;
    public static double linkageTarget = 0.3;

    public static boolean activateIntake = false;
    public static double intakeTarget = 0.45;

    public static boolean activateTurret = false;
    public static double turretTarget = 0;

    public static boolean activateClaw = false;
    public static double clawTarget = 0.5;

    public static boolean activateDeposit = false;
    public static double depositTarget = 0.5;

    public static boolean activateSlides = false;
    public static double slideTarget = 0;

    public static boolean activateAligner = false;
    public static double alignerTarget = 0.7;


    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

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

        //Initialize FTCDashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        //class that runs our linear slide
        LinearSlide slide = new LinearSlide(hardwareMap);
        slide.resetEncoders();

        TwoServo deposit = new TwoServo(depositRotationServoLeft,depositRotationServoRight);
        TwoServo intake = new TwoServo(inRotL,inRotR);
        Turret turret = new Turret(hardwareMap);

        Toggler right_trigger = new Toggler();
        Toggler right_bumper = new Toggler();
        Toggler left_bumper = new Toggler();

        //Fast loop go brrr
        PhotonCore.enable();

        waitForStart();
        while (opModeIsActive()) {

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
            depositTarget = (right_trigger.update(gamepad2.right_trigger > 0.1) ? Deposit.score : Deposit.transfer);

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
            else if (gamepad2.left_trigger > 0.1) {
                intakeTarget = 0.7;
            }

            turretTarget = Turret.zero;

            turret.moveTo(turretTarget);
            linkage.setPosition(linkageTarget);
            claw.setPosition(clawTarget);
            deposit.moveTo(depositTarget);
            intake.moveTo(intakeTarget);
            slide.update();
            telemetry.update();
        }
    }
}
