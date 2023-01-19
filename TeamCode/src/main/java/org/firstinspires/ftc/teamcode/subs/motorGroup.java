package org.firstinspires.ftc.teamcode.subs;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.maths.PIDcontroller;

public class motorGroup {

    private final DcMotorEx[] motors;
    private final PIDcontroller controller = new PIDcontroller(0,0,0,0);

    public motorGroup(DcMotorEx... motors){
        this.motors = motors;
    }

    public void setPIDgains(double Kp, double Kd, double Ki, double Kf){
        controller.setPIDCoeffs(Kp,Kd,Ki,Kf);
    }

    public void setPowers(double... powers){
        for (int i = 0; i < motors.length; i++){
            motors[i].setPower(powers[i]);
        }
    }

    public void setPositions(double target){
        setPowers(controller.out(target-getAveragePosition()));
    }

    public double getAveragePosition(){
        double state = 0;

        for (int i = 0; i < motors.length; i++){
            state += motors[i].getCurrentPosition();
        }
        return state /= motors.length;
    }


}
