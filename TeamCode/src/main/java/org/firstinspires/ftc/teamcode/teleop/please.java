package org.firstinspires.ftc.teamcode.teleop;

//Import EVERYTHING we need
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.hardware.lynx.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Linkage;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlide;


@Config
@TeleOp(name="please", group="Linear Opmode")
public class please extends LinearOpMode {

    public static double target = 1, maxVel = 1, maxAccel = 2, maxJerk = 3, Kp = 0, Kd = 0, Ki = 0, Kf = 0, limit = 1000;
    public static boolean up = true;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        Turret turret = new Turret(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {

            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();

            turret.moveToMP(target);
            turret.setMotionConstraints(maxVel, maxAccel, maxJerk);
            turret.setPIDcoeffs(Kp, Kd, Ki, Kf, limit);
            if (up) {
                intake.vertical();
            }
            else {
                intake.cone();
            }


            telemetry.addData("turret",turret.getHeading());
            telemetry.addData("turret2", turret.getMotionTarget());
            telemetry.update();
        }
    }
}
