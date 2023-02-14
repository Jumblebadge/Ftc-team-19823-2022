package org.firstinspires.ftc.teamcode.maths;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDcontroller {
    //PID controller class

    private double integralSum,out,lastError;
    private double Kp, Kd, Ki, Kf, Kv, Ka;
    private ElapsedTime timer = new ElapsedTime();
    private double integralLimit;

    public PIDcontroller(double Kp, double Kd, double Ki, double Kf, double integralLimit){
        this.Kp=Kp;
        this.Kd=Kd;
        this.Ki=Ki;
        this.Kf=Kf;
        this.integralLimit = integralLimit;
    }

    //calculate
    public double pidOut(double error) {
        if (Math.abs(error) > 0) {
            //integral and derivative values
            double derivative = (error - lastError) / timer.seconds();
            integralSum += (error * timer.seconds());
            integralSum = Range.clip(integralSum, -integralLimit, integralLimit);
            //weight each term so that tuning makes a difference
            out = (Kp * error) + (Kd * derivative) + (Ki * integralSum) + (Kf * Math.signum(error));
            out /= 10;
            lastError = error;
            timer.reset();
        }
        return out;
    }

    public double ffOut(double error, double velocityTarget, double accelerationTarget){
        return pidOut(error) + Kv * velocityTarget + Ka * accelerationTarget;
    }

    public void setPIDCoeffs(double Kp, double Kd, double Ki, double Kf, double limit){
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        this.Kf = Kf;
        this.integralLimit = limit;
    }

}
