package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.runMotionProfile;

public class twoServoBucket{

    private final Servo servo1, servo2;
    private final runMotionProfile profile = new runMotionProfile(0.1,0.1,0.1,0,0,0,0);
    private double lastTarget;

    public twoServoBucket(Servo servo2, Servo servo1){
        this.servo1 = servo1;
        this.servo2 = servo2;
    }

    public void moveToUsingProfile(double target){
        servo1.setPosition(profile.profiledServoMovement(target,servo1.getPosition()));
        servo2.setPosition(profile.profiledServoMovement(1-target,servo2.getPosition()));
    }

    public void moveTo(double target){
        if (lastTarget != target) {
            lastTarget = target;
            servo1.setPosition(1-target);
            servo2.setPosition(target);
        }
        else{ lastTarget = target;}
    }

    public void setMotionConstraints(double maxVel, double maxAccel, double maxJerk){
        profile.setMotionConstraints(maxVel, maxAccel, maxJerk);
    }

    public void setPIDcoeffs(double Kp, double Kd, double Ki, double Kf){
        profile.setPIDcoeffs(Kp,Kd,Ki,Kf);
    }

    public double getMotionTarget(){
        return profile.getMotionTarget();
    }

}
