package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.maths.PIDcontroller;
import org.firstinspires.ftc.teamcode.maths.SlewRateLimiter;

public class Turret {

    private final CRServoImplEx servo;
    private final AnalogInput ma3;
    private double adjust = 240, r = 5;
    private final PIDcontroller pid = new PIDcontroller(0.1,0,0,0, 100);
    private final SlewRateLimiter limiter = new SlewRateLimiter();

    public Turret(CRServoImplEx servo, AnalogInput ma3){
        this.servo = servo;
        this.ma3 = ma3;
    }

    public double getHeading(){
        return AngleUnit.normalizeDegrees(ma3.getVoltage() * 74.16 - adjust);
    }

    public void moveTo(double target){
        target = Range.clip(target, -90,90);
        servo.setPower(limiter.rateLimit(pid.pidOut(AngleUnit.normalizeDegrees(target - getHeading())), r));
    }

    public void setPIDcoeffs(double Kp, double Kd, double Ki, double Kf, double limit){
        pid.setPIDCoeffs(Kp, Kd, Ki, Kf, limit);
    }

    public void setR(double r){
        this.r = r;
    }

    public void setAdjust(double adjust){
        this.adjust = adjust;
    }
}
