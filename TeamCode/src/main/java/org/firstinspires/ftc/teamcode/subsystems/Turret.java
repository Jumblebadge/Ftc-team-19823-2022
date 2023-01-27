package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.maths.PIDcontroller;
import org.firstinspires.ftc.teamcode.maths.slewRateLimiter;

public class Turret {

    private final CRServoImplEx servo;
    private final AnalogInput ma3;
    private double adjust = -50, r = 3;
    private final PIDcontroller pid = new PIDcontroller(0.1,0,0,0);
    private final slewRateLimiter limiter = new slewRateLimiter();

    public Turret(CRServoImplEx servo, AnalogInput ma3){
        this.servo = servo;
        this.ma3 = ma3;
    }

    public double getHeading(){
        return AngleUnit.normalizeDegrees(ma3.getVoltage() * 74.16 - adjust);
    }

    public void moveTo(double target){
        target = Range.clip(target, -90,90);
        servo.setPower(limiter.rateLimit(pid.out(AngleUnit.normalizeDegrees(target - getHeading())), r));
    }

    public void setPIDcoeffs(double Kp, double Kd, double Ki, double Kf){
        pid.setPIDCoeffs(Kp, Kd, Ki, Kf);
    }

    public void setR(double r){
        this.r = r;
    }

    public void setAdjust(double adjust){
        this.adjust = adjust;
    }
}
