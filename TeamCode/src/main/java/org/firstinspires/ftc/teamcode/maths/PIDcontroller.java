package org.firstinspires.ftc.teamcode.maths;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDcontroller {
    //PID controller class

    private double integralSum,out,lastError;
    private double Kp, Kd, Ki, Kf;
    private ElapsedTime timer = new ElapsedTime();

    public PIDcontroller(double Kp, double Kd, double Ki, double Kf){
        this.Kp=Kp;
        this.Kd=Kd;
        this.Ki=Ki;
        this.Kf=Kf;
    }

    //calculate
    public double out(double error) {
        if (Math.abs(error) > 0) {
            //integral and derivative values
            double derivative = (error - lastError) / timer.seconds();
            integralSum = integralSum + (error * timer.seconds());
            //weight each term so that tuning makes a difference
            out = (Kp * error) + (Kd * derivative) + (Ki * integralSum) + (Kf * Math.signum(error));
            out /= 10;
            lastError = error;
            timer.reset();
        }
        return out;
    }

    public void setPIDCoeffs(double Kp, double Kd, double Ki, double Kf){
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        this.Kf = Kf;
    }
}
