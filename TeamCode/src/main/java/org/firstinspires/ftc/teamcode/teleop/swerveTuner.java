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
@TeleOp(name="swerveTuner", group="Linear Opmode")
public class swerveTuner extends LinearOpMode {

    public static double Kp = 0, Kd = 0, Ki = 0, Kf = 0, Kl = 10;
    public static double offset = 0;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        //class to swerve the swerve
        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap, true);

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        //Fast loop go brrr
        PhotonCore.enable();

        waitForStart();
        while (opModeIsActive()) {

            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();

            swerve.drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y,gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x);



            telemetry.update();
        }
    }
}
