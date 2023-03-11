package org.firstinspires.ftc.teamcode.teleop;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.HardwareDeviceManager;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.hardware.lynx.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.auto.cyclestuff;
import org.firstinspires.ftc.teamcode.maths.PIDcontroller;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlide;


//chub: 0 - deposit 1, 2 - deposit 2, 3 - claw, 4 - turret
//exhub: 0 - intake 1, 2 - intake 2, 4 - linkage, 5 - aligner

@Config
@TeleOp(name="godSwerve", group="Linear Opmode")
public class godSwerve extends LinearOpMode {

    enum cycleStates {
        INTAKE_GRAB,
        INTAKE_UP,
        DEPOSIT_EXTEND,
        DEPOSIT_DUMP,
        MANUAL,
        TRANSFER,
        DRIVE
    }

    please.cycleStates cyclestate = please.cycleStates.MANUAL;

    ElapsedTime timer = new ElapsedTime();

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
        ButtonDetector dpad_left     = new ButtonDetector();
        ButtonDetector button_b      = new ButtonDetector();
        ButtonDetector button_a      = new ButtonDetector();

        PIDcontroller headingPID = new PIDcontroller(6,0,5,0, 0.1);
        double headingOut, headingTarget = 0;

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        //Fast loop go brrr
        PhotonCore.enable();
        PhotonCore.experimental.setMaximumParallelCommands(8);

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

            if (button_b.constantUpdate(gamepad1.b)) {
                headingOut = headingPID.pidOut(AngleUnit.normalizeRadians(headingTarget - swerve.getHeading() * (Math.PI / 180)));
            }
            else {
                headingOut = 0;
            }

            swerve.drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y,headingOut + gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x);

            switch (cyclestate){
                case MANUAL:

                    linkage.toggle(right_bumper.constantUpdate(gamepad2.right_bumper));

                    claw.toggle(left_bumper.constantUpdate(gamepad2.left_bumper));

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

                    if (gamepad2.right_bumper && gamepad2.left_bumper) {
                        cyclestate = please.cycleStates.INTAKE_GRAB;
                    }
                    break;

                case DRIVE:
                    if (dpad_left.isolatedUpdate(gamepad2.dpad_left)) {
                        cyclestate = please.cycleStates.INTAKE_GRAB;
                    }

                    break;

                case INTAKE_GRAB:
                    slide.transfer();
                    intake.cone();
                    deposit.transfer();
                    claw.open();

                    if (dpad_left.isolatedUpdate(gamepad2.dpad_left)) {
                        cyclestate = please.cycleStates.INTAKE_UP;
                        timer.reset();
                    }
                    break;

                case INTAKE_UP:
                    claw.close();
                    if (timer.seconds() > 0.2) {
                        right_bumper.toFalse();
                        intake.transfer();
                    }

                    if (dpad_left.isolatedUpdate(gamepad2.dpad_left) || timer.seconds() > 0.7) {
                        cyclestate = please.cycleStates.TRANSFER;
                        timer.reset();
                    }
                    break;

                case TRANSFER:
                    claw.open();

                    if (dpad_left.isolatedUpdate(gamepad2.dpad_left) || timer.seconds() > 0.2) {
                        cyclestate = please.cycleStates.DEPOSIT_EXTEND;
                    }
                    break;

                case DEPOSIT_EXTEND:
                    intake.vertical();

                    if (dpad_left.isolatedUpdate(gamepad2.dpad_left)) {
                        cyclestate = please.cycleStates.DEPOSIT_DUMP;
                    }
                    break;

                case DEPOSIT_DUMP:
                    deposit.score();

                    if (dpad_left.isolatedUpdate(gamepad2.dpad_left)) {
                        cyclestate = please.cycleStates.DRIVE;
                        deposit.transfer();
                        slide.transfer();
                    }
                    break;
            }

            if (gamepad2.dpad_up && cyclestate != please.cycleStates.MANUAL) {
                cyclestate = please.cycleStates.MANUAL;
            }

            if (gamepad2.dpad_down && cyclestate != please.cycleStates.MANUAL) {
                cyclestate = please.cycleStates.INTAKE_GRAB;
            }


            if (gamepad2.a) {
                slide.zero(gamepad2.a);
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

            if (slide.returnPole() == LinearSlide.zero) {
                slide.toggleAligner(gamepad2.a);
            }



            if (gamepad1.a) {
                swerve.resetIMU();
            }

            turret.moveTo(0);
            slide.update();
            telemetry.addData("hz",1/hztimer.seconds());
            hztimer.reset();
            telemetry.update();
        }
    }
}
