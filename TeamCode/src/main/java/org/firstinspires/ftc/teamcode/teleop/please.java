package org.firstinspires.ftc.teamcode.teleop;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.lynx.*;
import com.acmerobotics.dashboard.*;

import org.firstinspires.ftc.teamcode.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.subsystems.TwoServo;

import java.util.List;

@Config
@TeleOp(name="please", group="Linear Opmode")
public class please extends LinearOpMode {

    //Initialize FTCDashboard
    FtcDashboard dashboard;
    public static double pos = 0.5;
    public static double Kp = 0, Kd = 0, Ki = 0, Kf = 0, limit = 1000;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Initialize FTCDashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ServoImplEx inL = hardwareMap.get(ServoImplEx.class, "inL");
        ServoImplEx inR = hardwareMap.get(ServoImplEx.class, "inR");

        TwoServo intake = new TwoServo(inL, inR);

        //good to go: ALIGNER, INTAKE L AND R, CLAW
        //to check: DEPOSIT L AND R, LINKAGE, TURRET

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


        //5 linkage, 4 turret, 3 intake right, 2 intake left, 1 aligner, 0 claw

        waitForStart();
        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            intake.moveTo(pos);

            telemetry.addData("pos",pos);
            telemetry.update();


        }
    }
}
