package org.firstinspires.ftc.teamcode.teleop;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.hardware.lynx.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlide;


@Config
@TeleOp(name="please", group="Linear Opmode")
public class please extends LinearOpMode {

    enum cycleStates {
        INTAKE_GRAB,
        INTAKE_UP,
        DEPOSIT_EXTEND,
        DEPOSIT_DUMP,
        MANUAL,
        TRANSFER,
        DRIVE
    }

    cycleStates cyclestate = cycleStates.MANUAL;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        //class to swerve the swerve
        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap, true);

        ElapsedTime timer = new ElapsedTime();

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
        ButtonDetector dpad_right    = new ButtonDetector();

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

            switch (cyclestate){
                case MANUAL:

                    //rising edge detector for claw open/close
                    if (left_bumper.constantUpdate(gamepad2.left_bumper)) {
                        claw.close();
                    } else {
                        claw.open();
                    }

                    //rising edge detector for outtake positions
                    if (right_trigger.constantUpdate(gamepad2.right_trigger > 0.1)) {
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
                    else if (dpad_left.isolatedUpdate(gamepad2.dpad_left)) {
                        intake.beacon();
                    }
                    else if (gamepad2.left_trigger > 0.1) {
                        intake.lowJunction();
                    }

                    if (gamepad2.right_bumper && gamepad2.left_bumper) {
                        cyclestate = cycleStates.INTAKE_GRAB;
                    }
                    break;

                case DRIVE:
                    if (dpad_left.isolatedUpdate(gamepad2.dpad_left)) {
                        cyclestate = cycleStates.INTAKE_GRAB;
                    }

                    break;

                case INTAKE_GRAB:
                    slide.transfer();
                    intake.cone();
                    deposit.transfer();
                    claw.open();

                    if (dpad_left.isolatedUpdate(gamepad2.dpad_left)) {
                        cyclestate = cycleStates.INTAKE_UP;
                        timer.reset();
                    }
                    break;

                case INTAKE_UP:
                    claw.close();
                    if (timer.seconds() > 0.1) {
                        right_bumper.toFalse();
                        intake.transfer();
                    }

                    if (dpad_right.isolatedUpdate(gamepad2.dpad_right)) {
                        cyclestate = cycleStates.INTAKE_GRAB;
                    }
                    if (dpad_left.isolatedUpdate(gamepad2.dpad_left) || timer.seconds() > 0.7) {
                        cyclestate = cycleStates.TRANSFER;
                        timer.reset();
                    }
                    break;

                case TRANSFER:
                    claw.open();

                    if (dpad_left.isolatedUpdate(gamepad2.dpad_left) || timer.seconds() > 0.2) {
                        cyclestate = cycleStates.DEPOSIT_EXTEND;
                    }
                    break;

                case DEPOSIT_EXTEND:
                    intake.vertical();

                    if (dpad_left.isolatedUpdate(gamepad2.dpad_left)) {
                        cyclestate = cycleStates.DEPOSIT_DUMP;
                    }
                    break;

                case DEPOSIT_DUMP:
                    deposit.score();

                    if (dpad_left.isolatedUpdate(gamepad2.dpad_left)) {
                        cyclestate = cycleStates.DRIVE;
                        deposit.transfer();
                        slide.transfer();
                    }
                    break;
            }

            if (gamepad2.dpad_up && cyclestate != cycleStates.MANUAL) {
                cyclestate = cycleStates.MANUAL;
            }

            if (gamepad2.dpad_down && cyclestate != cycleStates.MANUAL) {
                cyclestate = cycleStates.INTAKE_GRAB;
            }

            if (gamepad2.a) {
                slide.zero(!gamepad2.a);
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
            telemetry.addData("state",cyclestate.toString());
            turret.moveTo(Turret.zero);
            slide.update();
            telemetry.update();
        }
    }
}
