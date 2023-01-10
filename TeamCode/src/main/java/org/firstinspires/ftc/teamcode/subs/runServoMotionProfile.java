package org.firstinspires.ftc.teamcode.subs;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class runServoMotionProfile {

    private final Servo servo;
    private double lastTarget;
    private double maxVel,maxAccel,maxJerk;
    private ElapsedTime timer = new ElapsedTime();
    private MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0,0,0),new MotionState(1,0,0),1,1,1);

    public runServoMotionProfile(Servo servo){
        this.servo=servo;
    }

    public void setMotionConstraints(double maxVel, double maxAccel, double maxJerk){
        this.maxVel=maxVel;
        this.maxAccel=maxAccel;
        this.maxJerk=maxJerk;
    }

    public void runProfile(double target){
        if (lastTarget != target) {
            lastTarget = target;
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(servo.getPosition(), 0, 0), new MotionState(target, 0, 0), maxVel, maxAccel,maxJerk);
            timer.reset();
        }
        else{ lastTarget = target; }
        MotionState motionState = profile.get(timer.seconds());
        servo.setPosition(motionState.getX());
    }
}
