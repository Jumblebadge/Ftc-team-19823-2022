package org.firstinspires.ftc.teamcode.maths;

import com.qualcomm.robotcore.util.ElapsedTime;

public class controlLoopMath {

    private double integralSum,out,lastError;
    private double Kp, Kd, Ki, Kf;
    private ElapsedTime elapsedtime;

    public controlLoopMath(double Kp, double Kd, double Ki, double Kf, ElapsedTime elapsedtime){
        this.elapsedtime=elapsedtime;
        this.Kp=Kp;
        this.Kd=Kd;
        this.Ki=Ki;
        this.Kf=Kf;
    }
    //PID loop
    public double PIDout(double error) {
        if (Math.abs(error) > 0) {
            //calculate the integral and derivative values
            double derivative = (error - lastError) / elapsedtime.seconds();
            integralSum = integralSum + (error * elapsedtime.seconds());
            //multiplies those values by a constant, pre-tuned for whatever we are controlling
            out = (Kp * error) + (Kd * derivative) + (Ki * integralSum) + (Kf * Math.signum(error));
            out/=10;
            lastError = error;
            elapsedtime.reset();
        }
        return out;
    }
}
