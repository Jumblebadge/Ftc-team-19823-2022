package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;import com.acmerobotics.dashboard.config.Config;import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;import com.qualcomm.robotcore.hardware.DcMotorEx;
@Config
@TeleOp(name="noEncode", group="Linear Opmode")
public class noEncode extends LinearOpMode {private DcMotorEx mod1m2 = null, mod2m2 = null, mod3m2 = null;private DcMotorEx mod1m1 = null, mod2m1 = null, mod3m1 = null;
    public void runOpMode() {mod1m1 = hardwareMap.get(DcMotorEx.class, "mod1m1");mod2m1 = hardwareMap.get(DcMotorEx.class, "mod2m1");mod3m1 = hardwareMap.get(DcMotorEx.class, "mod3m1");mod1m2 = hardwareMap.get(DcMotorEx.class, "mod1m2");mod2m2 = hardwareMap.get(DcMotorEx.class, "mod2m2");mod3m2 = hardwareMap.get(DcMotorEx.class, "mod3m2");
        waitForStart();
        double j1 = 0;
        double j2 = 0;
        while (opModeIsActive()) {
            j1 = gamepad1.left_stick_y;
            j2 = gamepad1.right_stick_y;
            mod1m1.setPower(j1);mod1m2.setPower(j2);mod2m1.setPower(j1);mod2m2.setPower(j2);mod3m1.setPower(j1);mod3m2.setPower(j2);}}}