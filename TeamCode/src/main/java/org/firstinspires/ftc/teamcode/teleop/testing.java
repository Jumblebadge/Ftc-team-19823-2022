package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.wrappers.myElapsedTime;

@Config
@TeleOp(name="testing", group="Linear Opmode")
public class testing extends LinearOpMode {

    FtcDashboard dashboard;
    myElapsedTime timer = new myElapsedTime();
    public static boolean pause = false;
    public static boolean reset = false;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        dashboard = FtcDashboard.getInstance();

        PhotonCore.enable();

        waitForStart();
        while (opModeIsActive()) {

            if(pause){
                timer.pauseTimer();
            }
            else {
                timer.resumeTimer();
            }

            if (reset){
                timer.reset();
                reset = false;
            }

            telemetry.addData("time",timer.seconds());
            telemetry.update();
        }

    }

}