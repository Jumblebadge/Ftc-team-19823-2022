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

import org.firstinspires.ftc.teamcode.maths.PIDcontroller;
import org.firstinspires.ftc.teamcode.subsystems.linearSlide;
import org.firstinspires.ftc.teamcode.subsystems.twoServoBucket;
import org.firstinspires.ftc.teamcode.utility.myDcMotorEx;

import java.util.List;


@Config
@TeleOp(name="testing", group="Linear Opmode")
public class testing extends LinearOpMode {

    //Initialize FTCDashboard
    FtcDashboard dashboard;
    public static double depositTarget = 0.5, clawTarget = 0.5, linkageTarget = 0.5, intakeTarget = 0.5, slideTarget = 0;
    public static boolean update = false;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Initialize FTCDashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx liftLeftMotor = hardwareMap.get(DcMotorEx.class, "Llift");
        DcMotorEx liftRightMotor = hardwareMap.get(DcMotorEx.class,"Rlift");

        ServoImplEx depositRotationServoLeft = hardwareMap.get(ServoImplEx.class, "outL");
        ServoImplEx depositRotationServoRight = hardwareMap.get(ServoImplEx.class, "outR");
        ServoImplEx inRotL = hardwareMap.get(ServoImplEx.class,"inL");
        ServoImplEx inRotR = hardwareMap.get(ServoImplEx.class,"inR");
        ServoImplEx linkage = hardwareMap.get(ServoImplEx.class, "linkage");
        CRServoImplEx turret = hardwareMap.get(CRServoImplEx.class, "turret");
        Servo claw = hardwareMap.get(Servo.class, "claw");
        depositRotationServoLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
        depositRotationServoRight.setPwmRange(new PwmControl.PwmRange(500, 2500));
        inRotL.setPwmRange(new PwmControl.PwmRange(500,2500));
        inRotR.setPwmRange(new PwmControl.PwmRange(500,2500));
        linkage.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turret.setPwmRange(new PwmControl.PwmRange(500, 2500));

        //liftRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        //Initialize FTCDashboard
        dashboard = FtcDashboard.getInstance();

        //class that runs our linear slide
        linearSlide slide = new linearSlide(liftLeftMotor,liftRightMotor);
        slide.resetEncoders();

        twoServoBucket deposit = new twoServoBucket(depositRotationServoLeft,depositRotationServoRight);
        twoServoBucket intake = new twoServoBucket(inRotL,inRotR);

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

            //turret.setPower(turretPID.out(turretTarget - turretState));
            turret.setPower(gamepad1.right_stick_y);
            deposit.moveTo(depositTarget);
            intake.moveTo(intakeTarget);
            if(update){
                linkage.setPosition(linkageTarget);
                claw.setPosition(clawTarget);
            }
            slide.moveTo(slideTarget);



        }
    }
}
