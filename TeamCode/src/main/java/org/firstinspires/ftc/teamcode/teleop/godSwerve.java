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
@TeleOp(name="godSwerve", group="Linear Opmode")
public class godSwerve extends LinearOpMode {

    //Tuning values so that wheels are always facing straight (accounts for encoder drift - tuned manually)
    public static double mod3PC = -20, mod1PC = -20, mod2PC = -105;
    public static double Kp = 0, Kd = 0, Ki = 0, Kf = 0, limit = 1000;

    double depositTarget = 0.3, clawTarget = 0.5, linkageTarget = 0.5, intakeTarget = 0.5, slideTarget = 0, turretTarget = 0, alignerTarget = 0.75;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Initialize FTCDashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        //Initialize FTCDashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        //class to swerve the swerve
        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap, true);

        ElapsedTime hztimer = new ElapsedTime();

        //class that runs our linear slide
        LinearSlide slide = new LinearSlide(hardwareMap);
        slide.resetEncoders();

        Deposit deposit = new Deposit(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Turret turret = new Turret(hardwareMap);
        Linkage linkage = new Linkage(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        Toggler right_trigger = new Toggler();
        Toggler right_bumper = new Toggler();
        Toggler left_bumper = new Toggler();

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        //Fast loop go brrr
        PhotonCore.enable();

        //linkage.setPosition(init);
        //claw.setPosition(0.6);
        //outRotL.setPosition(1);
        //outRotR.setPosition(1-outRotL.getPosition());
        //inRotL.setPosition(0.3);
        //inRotR.setPosition(1-inRotL.getPosition());

        waitForStart();
        while (opModeIsActive()) {

            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();

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

            if (right_bumper.update(gamepad2.right_bumper)) {
                linkage.out();
            } else {
                linkage.in();
            }

            //rising edge detector for claw open/close
            if (left_bumper.update(gamepad2.left_bumper)) {
                claw.close();
            } else {
                claw.open();
            }

            //rising edge detector for outtake positions
            if (right_trigger.update(gamepad2.right_trigger > 0.1)) {
                deposit.score();
            } else {
                deposit.transfer();
            }

            if(gamepad2.dpad_right){
                intake.vertical();
            }
            else if (gamepad2.dpad_down){
                intake.transfer();
            }
            else if (gamepad2.dpad_up){
                intake.cone();
            }
            else if (gamepad2.dpad_left) {
                intake.beacon();
            }
            else if (gamepad2.left_trigger > 0.1) {
                intake.lowJunction();
            }

            turretTarget = Turret.zero;

            turret.moveTo(turretTarget);
            slide.update();
            swerve.rotateKids(AngleUnit.normalizeDegrees(cyclestuff.heading));

            telemetry.addData("turrettarget",turretTarget);
            telemetry.addData("slidetarget",slideTarget);;
            telemetry.addData("hz",1/hztimer.seconds());
            hztimer.reset();
            telemetry.update();
        }
    }
}
