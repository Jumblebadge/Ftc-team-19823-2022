package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.maths.PIDcontroller;
import org.firstinspires.ftc.teamcode.maths.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.utility.RunMotionProfile;

public class Turret {

    private final CRServoImplEx servo;
    private final AnalogInput ma3;
    private double adjust = 240, r = 5;
    private final PIDcontroller pid = new PIDcontroller(0.1,0.001,0.1,0, 0.25);
    private final RunMotionProfile profile = new RunMotionProfile(1,2,3,0,0,0,0,1000);
    private final SlewRateLimiter limiter = new SlewRateLimiter();
    public static final double zero = 0, stackPickup = 56;

    public Turret(HardwareMap hardwareMap){
        servo = hardwareMap.get(CRServoImplEx.class, "turret");
        ma3 = hardwareMap.get(AnalogInput.class, "turretMa3");
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public double getHeading(){
        return AngleUnit.normalizeDegrees(ma3.getVoltage() * 74.16 - adjust);
    }

    public void moveTo(double target){
        target = Range.clip(target, -90,90);
        servo.setPower(limiter.rateLimit(pid.pidOut(AngleUnit.normalizeDegrees(target - getHeading())), r));
    }

    public void moveToMP(double target) {
        target = Range.clip(target, -90,90);
        servo.setPower(profile.profiledMovement(target, getHeading()));
    }

    public void PWMrelease() {
        servo.setPwmDisable();
    }

    public void setMotionConstraints(double maxVel, double maxAccel, double maxJerk){
        profile.setMotionConstraints(maxVel, maxAccel, maxJerk);
    }

    public void setPIDcoeffs(double Kp, double Kd, double Ki, double Kf, double limit){
        profile.setPIDcoeffs(Kp, Kd, Ki, Kf, limit);
    }

    public double getMotionTarget(){
        return profile.getMotionTarget();
    }

    public void setR(double r){
        this.r = r;
    }

    public void setAdjust(double adjust){
        this.adjust = adjust;
    }
}
