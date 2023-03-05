package org.firstinspires.ftc.teamcode.teleop;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.hardware.lynx.*;
import com.acmerobotics.dashboard.*;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlide;


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

        ButtonDetector right_trigger = new ButtonDetector();

        //Initialize FTCDashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {

            controlHub.clearBulkCache();


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


            slide.moveTo(slideTarget);
            telemetry.addData("slide",-slide.getPosition());
            telemetry.addData("slidetar",slide.getMotionTarget());
            telemetry.update();
        }
    }
}
