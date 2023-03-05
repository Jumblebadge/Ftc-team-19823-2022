package org.firstinspires.ftc.teamcode.teleop;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.hardware.lynx.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.auto.cyclestuff;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlide;


@Config
@TeleOp(name="godSwerve", group="Linear Opmode")
public class godSwerve extends LinearOpMode {

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        //class to swerve the swerve
        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap, true);

        ElapsedTime hztimer = new ElapsedTime();

        //class that runs our linear slide
        LinearSlide slide = new LinearSlide(hardwareMap);
        slide.resetEncoders();

        Deposit deposit = new Deposit(hardwareMap);
        Intake intake   = new Intake(hardwareMap);
        Turret turret   = new Turret(hardwareMap);
        Linkage linkage = new Linkage(hardwareMap);
        Claw claw       = new Claw(hardwareMap);

        ButtonDetector right_trigger = new ButtonDetector();
        ButtonDetector right_bumper  = new ButtonDetector();
        ButtonDetector left_bumper   = new ButtonDetector();

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

            linkage.toggle(right_bumper.constantUpdate(gamepad2.right_bumper));

            claw.toggle(right_bumper.constantUpdate(gamepad2.right_bumper));

            deposit.toggle(right_trigger.constantUpdate(gamepad2.right_trigger > 0.1));

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

            double turretTarget = Turret.zero;

            turret.moveTo(turretTarget);
            slide.update();
            swerve.rotateKids(AngleUnit.normalizeDegrees(cyclestuff.heading));

            telemetry.addData("hz",1/hztimer.seconds());
            hztimer.reset();
            telemetry.update();
        }
    }
}
