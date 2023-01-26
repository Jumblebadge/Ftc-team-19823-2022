package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.maths.PIDcontroller;

public class Turret {

    private final CRServoImplEx servo;
    private final AnalogInput ma3;
    private double adjust;
    private double Kp, Kd, Ki, Kf;
    private final PIDcontroller pid = new PIDcontroller(0,0,0,0);

    public Turret(CRServoImplEx servo, AnalogInput ma3){
        this.servo = servo;
        this.ma3 = ma3;
    }

    public double getHeading(){
        return ma3.getVoltage() * 74.16 - adjust;
    }

    public void moveTo(double target){
        servo.setPower(pid.out(AngleUnit.normalizeDegrees(target - getHeading())));
    }

    public void setPIDcoeffs(double Kp, double Kd, double Ki, double Kf){
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        this.Kf = Kf;
    }

    public void setAdjust(double adjust){
        this.adjust = adjust;
    }
}
