package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="testing", group="Linear Opmode")
public class testing extends LinearOpMode {

    FtcDashboard dashboard;
    public static double clawRotTarget = 0.5;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo clawrotL = hardwareMap.get(Servo.class,"clawrotL");
        Servo clawrotR = hardwareMap.get(Servo.class,"clawrotR");
        dashboard = FtcDashboard.getInstance();

        PhotonCore.enable();

        waitForStart();
        while (opModeIsActive()) {

            clawrotL.setPosition(clawRotTarget);
            clawrotR.setPosition(1-clawRotTarget);

            telemetry.update();
        }

    }

}