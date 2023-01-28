package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.maths.PIDcontroller;

public class runMotionProfile {

    private double lastTarget;
    private double maxVel, maxAccel, maxJerk;
    private MotionState motionState;
    private final PIDcontroller PID;
    private final ElapsedTime timer = new ElapsedTime();
    private MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0,0,0),new MotionState(1,0,0),1,1,1);

    public runMotionProfile(double maxVel, double maxAccel, double maxJerk, double Kp, double Kd, double Ki, double Kf){
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxJerk = maxJerk;

        PID = new PIDcontroller(Kp,Kd,Ki,Kf);
    }

    public void setMotionConstraints(double maxVel, double maxAccel, double maxJerk){
        this.maxVel=maxVel;
        this.maxAccel=maxAccel;
        this.maxJerk=maxJerk;
    }

    public void setPIDcoeffs(double Kp, double Kd, double Ki, double Kf){ PID.setPIDCoeffs(Kp,Kd,Ki,Kf); }

    public double profiledMovement(double target, double state){
        if (lastTarget != target) {
            lastTarget = target;
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(state, 0, 0), new MotionState(target, 0, 0), maxVel, maxAccel,maxJerk);
            timer.reset();
        }
        else{ lastTarget = target; }
        motionState = profile.get(timer.seconds());
        return PID.pidOut(motionState.getX()-state);
    }

    public double getMotionTarget(){
        return motionState.getX();
    }

    public double profiledServoMovement(double target, double state){
        if (lastTarget != target) {
            lastTarget = target;
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(state, 0, 0), new MotionState(target, 0, 0), maxVel, maxAccel,maxJerk);
            timer.reset();
        }
        else{ lastTarget = target; }
        motionState = profile.get(timer.seconds());
        return motionState.getX();
    }
}
