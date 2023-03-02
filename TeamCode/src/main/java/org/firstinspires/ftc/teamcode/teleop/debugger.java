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
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Linkage;
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

    public static double maxVel = 0, maxAccel = 0, maxJerk = 0;


    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        //class that runs our linear slide
        LinearSlide slide = new LinearSlide(hardwareMap);
        slide.resetEncoders();

        Deposit deposit = new Deposit(hardwareMap);
        Intake intake   = new Intake(hardwareMap);
        Turret turret   = new Turret(hardwareMap);
        Linkage linkage = new Linkage(hardwareMap);
        Claw claw       = new Claw(hardwareMap);

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        //Fast loop go brrr
        PhotonCore.enable();

        Toggler right_trigger = new Toggler();

        //Initialize FTCDashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {

            controlHub.clearBulkCache();

            if (right_trigger.update(gamepad2.right_trigger > 0.1)) {
                deposit.score();
            } else {
                deposit.transfer();
            }

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

            if (activateTurret) {
                turret.moveTo(turretTarget);
            }
            if (activateLinkage) {
                linkage.moveTo(linkageTarget);
            }
            if (activateClaw) {
                claw.moveTo(clawTarget);
            }
            if (activateDeposit) {
                deposit.moveTo(depositTarget);
            }
            if (activateIntake) {
                intake.moveTo(intakeTarget);
            }
            if (activateSlides) {
                slide.update();
            }

            //slide.moveTo(slideTarget);
            telemetry.addData("slide",-slide.getPosition());
            telemetry.addData("slidetar",slide.getMotionTarget());
            telemetry.update();
        }
    }
}
