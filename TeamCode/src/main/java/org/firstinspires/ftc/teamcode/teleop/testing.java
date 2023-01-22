package org.firstinspires.ftc.teamcode.teleop;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.lynx.*;
import com.acmerobotics.dashboard.*;

import org.firstinspires.ftc.teamcode.subsystems.linearSlide;
import org.firstinspires.ftc.teamcode.subsystems.twoServoBucket;

import java.util.List;


@Config
@TeleOp(name="testing", group="Linear Opmode")
public class testing extends LinearOpMode {

    //Initialize FTCDashboard
    FtcDashboard dashboard;

    public static double Kp=0,Kd=0,Ki=0,Kf = 0,maxVel=0.1,maxAccel=0.1,maxJerk=0.1, liftTarget = 0, depositTarget = 0.5;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Initialize FTCDashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx liftLeftMotor = hardwareMap.get(DcMotorEx.class,"Llift");
        DcMotorEx liftRightMotor = hardwareMap.get(DcMotorEx.class,"Rlift");

        Servo depositRotationServoLeft = hardwareMap.get(ServoImplEx.class, "LdepositServo");
        Servo depositRotationServoRight = hardwareMap.get(ServoImplEx.class, "RdepositServo");

        liftRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        //Initialize FTCDashboard
        dashboard = FtcDashboard.getInstance();

        //class that runs our linear slide
        linearSlide slide = new linearSlide(liftLeftMotor,liftRightMotor);
        slide.resetEncoders();

        twoServoBucket deposit = new twoServoBucket(depositRotationServoLeft,depositRotationServoRight);

        //Bulk sensor reads
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //Fast loop go brrr
        PhotonCore.enable();

        waitForStart();
        while (opModeIsActive()) {

            deposit.setMotionConstraints(maxVel,maxAccel,maxJerk);

            //Clear the cache for better loop times (bulk sensor reads)
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            slide.moveTo(liftTarget);
            deposit.moveTo(depositTarget);

            telemetry.addData("motionTarget",deposit.getMotionTarget());
            telemetry.update();
        }
    }
}
