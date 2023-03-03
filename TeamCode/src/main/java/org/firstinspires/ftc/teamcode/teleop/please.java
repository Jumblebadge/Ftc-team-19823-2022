//package org.firstinspires.ftc.teamcode.teleop;
//
////Import EVERYTHING we need
//import com.acmerobotics.dashboard.config.Config;
//import com.outoftheboxrobotics.photoncore.PhotonCore;
//import com.qualcomm.robotcore.eventloop.opmode.*;
//import com.qualcomm.hardware.lynx.*;
//
//import org.firstinspires.ftc.robotcore.external.navigation.*;
//import org.firstinspires.ftc.teamcode.auto.cyclestuff;
//import org.firstinspires.ftc.teamcode.maths.mathsOperations;
//import org.firstinspires.ftc.teamcode.subsystems.Claw;
//import org.firstinspires.ftc.teamcode.subsystems.Deposit;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Linkage;
//import org.firstinspires.ftc.teamcode.subsystems.Turret;
//import org.firstinspires.ftc.teamcode.utility.Toggler;
//import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
//import org.firstinspires.ftc.teamcode.subsystems.LinearSlide;
//
//
//@Config
//@TeleOp(name="please", group="Linear Opmode")
//public class please extends LinearOpMode {
//
//    enum cycleStates {
//        INTAKE_GRAB,
//        INTAKE_UP,
//        DEPOSIT_EXTEND,
//        DEPOSIT_DUMP,
//        MANUAL,
//        TRANSFER
//    }
//
//    cycleStates cyclestate = cycleStates.MANUAL;
//
//    public void runOpMode() {
//        telemetry.addData("Status", "Initialized");
//
//        //Bulk sensor reads
//        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
//
//        //class to swerve the swerve
//        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap, true);
//
//        //class that runs our linear slide
//        LinearSlide slide = new LinearSlide(hardwareMap);
//        slide.resetEncoders();
//
//        Deposit deposit = new Deposit(hardwareMap);
//        Intake intake   = new Intake(hardwareMap);
//        Turret turret   = new Turret(hardwareMap);
//        Linkage linkage = new Linkage(hardwareMap);
//        Claw claw       = new Claw(hardwareMap);
//
//        Toggler right_trigger = new Toggler();
//        Toggler right_bumper  = new Toggler();
//        Toggler left_bumper   = new Toggler();
//
//        //Bulk sensor reads
//        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//
//        //Fast loop go brrr
//        PhotonCore.enable();
//
//        //linkage.setPosition(init);
//        //claw.setPosition(0.6);
//        //outRotL.setPosition(1);
//        //outRotR.setPosition(1-outRotL.getPosition());
//        //inRotL.setPosition(0.3);
//        //inRotR.setPosition(1-inRotL.getPosition());
//
//        waitForStart();
//        while (opModeIsActive()) {
//
//            //Clear the cache for better loop times (bulk sensor reads)
//            controlHub.clearBulkCache();
//
//            swerve.drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, mathsOperations.power(gamepad1.right_stick_x, 3));
//
//            switch (cyclestate){
//                case MANUAL:
//                    if (gamepad2.a) {
//                        slide.zero(gamepad2.left_trigger > 0.1);
//                    }
//                    else if (gamepad2.b) {
//                        slide.transfer();
//                    }
//                    else if (gamepad2.x) {
//                        slide.mediumPole();
//                    }
//                    else if (gamepad2.y) {
//                        slide.highPole();
//                    }
//
//                    if (right_bumper.update(gamepad2.right_bumper)) {
//                        linkage.out();
//                    } else {
//                        linkage.in();
//                    }
//
//                    //rising edge detector for claw open/close
//                    if (left_bumper.update(gamepad2.left_bumper)) {
//                        claw.close();
//                    } else {
//                        claw.open();
//                    }
//
//                    //rising edge detector for outtake positions
//                    if (right_trigger.update(gamepad2.right_trigger > 0.1)) {
//                        deposit.score();
//                    } else {
//                        deposit.transfer();
//                    }
//
//                    if(gamepad2.dpad_right){
//                        intake.vertical();
//                    }
//                    else if (gamepad2.dpad_down){
//                        intake.transfer();
//                    }
//                    else if (gamepad2.dpad_up){
//                        intake.cone();
//                    }
//                    else if (gamepad2.dpad_left) {
//                        intake.beacon();
//                    }
//                    else if (gamepad2.left_trigger > 0.1) {
//                        intake.lowJunction();
//                    }
//
//                    if (gamepad2.dpad_up) {
//
//                    }
//                    break;
//
//                case INTAKE_GRAB:
//                    slide.transfer();
//                    deposit.moveTo(0.175);
//                    intake.moveTo(0.995-((5-cyclesCompleted)*0.0255));
//                    goofytimer.reset();
//                    autogoofytimer.reset();
//                    cyclestate = cycleStates.INTAKE_UP;
//                    break;
//
//                case INTAKE_UP:
//                    if (autogoofytimer.seconds() > 0.75){
//                        claw.setPosition(0.2);
//                    }
//                    if (goofytimer.seconds() > 1.3) {
//                        linkage.setPosition(0.25);
//                        intake.moveTo(0.3);
//                    }
//                    if (goofytimer.seconds() > 1.4) {
//                        turretTarget = 0;
//                        cyclestate = cyclestuff.cycleStates.TRANSFER;
//                        goofytimer.reset();
//                        autogoofytimer.reset();
//                    }
//                    break;
//
//                case TRANSFER:
//                    if (autogoofytimer.seconds() > 0.7) {
//                        claw.setPosition(0.5);
//                    }
//                    if (goofytimer.seconds() > 1.25) {
//                        intake.moveTo(0.45);
//                        cyclestate = cyclestuff.cycleStates.DEPOSIT_EXTEND;
//                    }
//                    break;
//
//                case DEPOSIT_EXTEND:
//                    //lift slides, drop cone, come back down
//                    turretTarget = Turret.stackPickup;
//                    linkage.setPosition(0.51);
//                    slide.highPole();
//                    if (slide.isTimeDone()) {
//                        cyclestate = cyclestuff.cycleStates.DEPOSIT_DUMP;
//                        goofytimer.reset();
//                    }
//
//                    break;
//
//                case DEPOSIT_DUMP:
//                    if (goofytimer.seconds() > 0.5) {
//                        deposit.moveTo(0.8);
//                    }
//                    if (goofytimer.seconds() > 1.325){
//                        deposit.moveTo(0.3);
//                        cyclesCompleted += 1;
//                        cyclestate = cyclestuff.cycleStates.INTAKE_GRAB;
//                    }
//                    break;
//            }
//
//            if (gamepad2.dpad_right && cyclestate != cycleStates.MANUAL) {
//                cyclestate = cycleStates.MANUAL;
//            }
//
//            turret.moveTo(Turret.zero);
//            slide.update();
//            telemetry.update();
//        }
//    }
//}
