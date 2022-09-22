package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.maths.controlLoopMath;
import org.firstinspires.ftc.teamcode.maths.mathsOperations;


@Config
@TeleOp(name="testing", group="Linear Opmode")
public class testing extends LinearOpMode {


    FtcDashboard dashboard;
    private DcMotorEx m1 = null, m2 = null;

    private AnalogInput mod1E = null;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        m1 = hardwareMap.get(DcMotorEx.class, "m1");
        m2 = hardwareMap.get(DcMotorEx.class,"m2");

        mod1E = hardwareMap.get(AnalogInput.class, "mod1E");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        dashboard = FtcDashboard.getInstance();

        ElapsedTime mod1timer =  new ElapsedTime();
        boolean mod1wrapped = false;
        double mod1lastpos = 0;
        double mod1P = 0;
        double reference = 0;
        controlLoopMath mod1PID = new controlLoopMath(0.2,0.0001,0,0,mod1timer);

        waitForStart();
        while (opModeIsActive()) {

            double mod1P1 = mod1E.getVoltage() * 74.16;

            double mod1positiondelta = mod1P1 - mod1lastpos;
            mod1lastpos = mod1P1;

            mod1wrapped = ((mod1positiondelta > 180) != mod1wrapped);
            mod1wrapped = ((mod1positiondelta <-180) != mod1wrapped);
            mod1P = (mod1wrapped == true ? 180 + mod1P1/2 : mod1P1/2);

            boolean simple = false;

            if(simple) {
                double[] values = mathsOperations.diffyConvert(gamepad1.right_stick_x,gamepad1.left_stick_y);
                m1.setPower(values[0]);
                m2.setPower(values[1]);
            }
            else {
                reference += gamepad1.left_stick_x;

                double[] values = mathsOperations.diffyConvert(mod1PID.PIDout(reference,mod1P),gamepad1.right_stick_y);
                m1.setPower(values[0]);
                m2.setPower(values[1]);
            }

            telemetry.addData("delta",mod1positiondelta);
            telemetry.addData("iswrapped?",mod1wrapped);
            telemetry.addData("mod1before",mod1P1);
            telemetry.addData("mod1after",mod1P);
            telemetry.addData("reference",reference);
            telemetry.update();

        }

    }

}
