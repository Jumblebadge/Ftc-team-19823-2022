package org.firstinspires.ftc.teamcode.teleop;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.lynx.*;
import com.acmerobotics.dashboard.*;

import org.firstinspires.ftc.teamcode.maths.mathsOperations;
import org.firstinspires.ftc.teamcode.maths.slewRateLimiter;
import org.firstinspires.ftc.teamcode.utility.Toggler;

import java.util.List;


@Config
@TeleOp(name="please", group="Linear Opmode")
public class please extends LinearOpMode {

    //Initialize FTCDashboard
    FtcDashboard dashboard;
    public static double r = 0;
    double pos = 0;

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
        Toggler right_bumper = new Toggler();

        waitForStart();
        while (opModeIsActive()) {

            pos = right_bumper.update(gamepad2.right_bumper) ? 0.7 : 0.3;

            telemetry.addData("pos",pos);
            telemetry.addData("last",right_bumper.last);
            telemetry.addData("current",right_bumper.current);
            telemetry.update();
        }
    }
}
