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
import org.firstinspires.ftc.teamcode.utility.Toggler;

import java.util.List;

@Config
@TeleOp(name="please", group="Linear Opmode")
public class please extends LinearOpMode {

    //Initialize FTCDashboard
    FtcDashboard dashboard;
    double slideTarget;
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Initialize FTCDashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        //Initialize FTCDashboard
        dashboard = FtcDashboard.getInstance();

        //Bulk sensor reads
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //Fast loop go brrr
        PhotonCore.enable();

        DcMotorEx liftLeftMotor = hardwareMap.get(DcMotorEx.class, "Llift");
        DcMotorEx liftRightMotor = hardwareMap.get(DcMotorEx.class,"Rlift");

        ServoImplEx depositRotationServoLeft = hardwareMap.get(ServoImplEx.class, "outL");
        ServoImplEx depositRotationServoRight = hardwareMap.get(ServoImplEx.class, "outR");
        depositRotationServoLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
        depositRotationServoRight.setPwmRange(new PwmControl.PwmRange(500, 2500));
        twoServoBucket deposit = new twoServoBucket(depositRotationServoLeft,depositRotationServoRight);
        Toggler right_trigger = new Toggler();
        liftLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        linearSlide slide = new linearSlide(liftLeftMotor,liftRightMotor);
        slide.resetEncoders();


        waitForStart();
        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            if (gamepad2.a) {
                slideTarget = 0;
            }
            else if (gamepad2.b) {
                slideTarget = 250;
            }
            else if (gamepad2.x) {
                slideTarget = 600;
            }
            else if (gamepad2.y) {
                slideTarget = 1000;
            }
            slide.moveTo(slideTarget);

            deposit.moveTo(right_trigger.update(gamepad2.right_trigger > 0.1) ? 0.275 : 0.85);


        }
    }
}
