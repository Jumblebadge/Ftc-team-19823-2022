package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileBuilder;
import com.acmerobotics.roadrunner.profile.MotionSegment;
import com.acmerobotics.roadrunner.profile.AccelerationConstraint;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.maths.controlLoopMath;
import org.firstinspires.ftc.teamcode.maths.mathsOperations;


@Config
@TeleOp(name="testing", group="Linear Opmode")
public class testing extends LinearOpMode {

    FtcDashboard dashboard;

    public static double target = 1;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        DcMotorEx lift = hardwareMap.get(DcMotorEx.class, "lift");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        dashboard = FtcDashboard.getInstance();

        ElapsedTime liftime = new ElapsedTime();
        ElapsedTime protime = new ElapsedTime();

        PhotonCore.enable();

        double lastTarget = 0;
        MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0,0,0),new MotionState(1,0,0),1,1);

        waitForStart();
        while (opModeIsActive()) {

            controlLoopMath liftPID = new controlLoopMath(0.2,0,0,0,liftime);
            if (lastTarget != target) {
                lastTarget = target;
                profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(lift.getCurrentPosition(), 0, 0), new MotionState(target, 0, 0), 2900, 2900);
                protime.reset();
            }
            else{ lastTarget = target; }

            MotionState state = profile.get(protime.seconds());
            double target1 = state.getX();
            double PIDOUT = liftPID.PIDout(target1-lift.getCurrentPosition());
            lift.setPower(PIDOUT);
            //if (Math.abs(target-lift.getCurrentPosition())<10) protime.reset();

            telemetry.addData("lift posisiton", lift.getCurrentPosition());
            telemetry.addData("lift target", target);
            telemetry.addData("motiontarget",target1);
            telemetry.addData("lasttarget",lastTarget);
            telemetry.addData("power out",PIDOUT);
            telemetry.update();
        }

    }

}
