package org.firstinspires.ftc.teamcode.teleop;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.lynx.*;
import com.acmerobotics.dashboard.*;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.maths.mathsOperations;
import org.firstinspires.ftc.teamcode.maths.slewRateLimiter;
import org.firstinspires.ftc.teamcode.utility.Toggler;

import java.util.List;


@Config
@TeleOp(name="please", group="Linear Opmode")
public class please extends LinearOpMode {

    //Initialize FTCDashboard
    FtcDashboard dashboard;
    public static double r = 0, div = 1;
    double pos = 0, pos2 = 0, pos3 = 0, pos4 = 0;

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

        slewRateLimiter limiter = new slewRateLimiter();
        slewRateLimiter limiter2 = new slewRateLimiter();

        waitForStart();
        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            pos = limiter.rateLimit(gamepad2.right_stick_x * 90, r);
            pos2 += gamepad2.right_stick_x/div;
            if(gamepad2.right_stick_button) { pos2 = 0; }
            pos3 += limiter2.rateLimit(gamepad2.right_stick_x, r)/div;
            pos4 += (gamepad2.right_stick_x*gamepad2.right_stick_x* gamepad2.right_stick_x)/div;

            telemetry.addData("pos",pos);
            telemetry.addData("pos2",pos2);
            telemetry.addData("pos3", pos3);
            telemetry.addData("pos4",pos4);
            telemetry.addData("last",gamepad2.right_stick_x);
            telemetry.update();
        }
    }
}