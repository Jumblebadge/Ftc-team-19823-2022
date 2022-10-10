package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

    private DcMotorEx m1 = null, m2 = null;

    private AnalogInput mod1E = null;

    public static double Kp = 0.1,Kd = 0.0001,Ki = 0.0007,Kf,reference;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        m1 = hardwareMap.get(DcMotorEx.class, "m1");
        m2 = hardwareMap.get(DcMotorEx.class,"m2");

        m2.setDirection(DcMotorSimple.Direction.REVERSE);

        mod1E = hardwareMap.get(AnalogInput.class, "mod1E");

        dashboard = FtcDashboard.getInstance();

        PhotonCore.enable();

        ElapsedTime mod1timer =  new ElapsedTime();
        boolean mod1wrapped = false;
        double mod1lastpos = 0;
        double mod1P = 0;
        reference = 0;
        //controlLoopMath mod1PID = new controlLoopMath(Kp,Kd,Ki,Kf,mod1timer);

        waitForStart();
        while (opModeIsActive()) {

            double mod1P1 = mod1E.getVoltage() * -74.16;

            controlLoopMath mod1PID = new controlLoopMath(Kp,Kd,Ki,Kf,mod1timer);

            double mod1positiondelta = mod1P1 - mod1lastpos;
            mod1lastpos = mod1P1;

            mod1wrapped = ((mod1positiondelta > 180) != mod1wrapped);
            mod1wrapped = ((mod1positiondelta <-180) != mod1wrapped);
            mod1P = (mod1wrapped == true ? 180 + mod1P1/2 : mod1P1/2);

            mod1P = mathsOperations.angleWrap(mod1P);

            double[] values;
            double atan = Math.atan2(-gamepad1.left_stick_x,-gamepad1.left_stick_y);
            atan *= 57.295779513082320876;
            reference = atan;

            if(gamepad1.a) {
                values = mathsOperations.diffyConvert(gamepad1.right_stick_x,gamepad1.left_stick_y);
            }
            else {
                double pid = mod1PID.PIDout(reference,mod1P);
                values = mathsOperations.diffyConvert(pid,0);
            }
            m1.setPower(values[0]);
            m2.setPower(values[1]);


            telemetry.addData("mod1pos",mod1P);
            telemetry.addData("reference",reference);
            telemetry.addData("pidout",mod1PID.PIDout(reference,mod1P));
            telemetry.addData("error",AngleUnit.normalizeDegrees(reference - mod1P));
            telemetry.addData("m1",values[0]);
            telemetry.addData("m2",values[1]);
            telemetry.update();

        }

    }

}
