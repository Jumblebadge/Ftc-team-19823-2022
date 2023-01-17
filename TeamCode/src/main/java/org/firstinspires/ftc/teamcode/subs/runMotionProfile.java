package org.firstinspires.ftc.teamcode.subs;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.maths.PIDcontroller;

public class runMotionProfile {

    private double lastTarget;
    private double maxVel, maxAccel, maxJerk;
    private double Kp, Kd, Ki, Kf;
    private final PIDcontroller PID;
    private final ElapsedTime timer = new ElapsedTime();
    private MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0,0,0),new MotionState(1,0,0),1,1,1);

    public runMotionProfile(double maxVel, double maxAccel, double maxJerk, double Kp, double Kd, double Ki, double Kf){
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxJerk = maxJerk;

        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        this.Kf = Kf;

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
        MotionState motionState = profile.get(timer.seconds());
        return PID.out(motionState.getX()-state);
    }

    public double profiledServoMovement(double target, double state){
        if (lastTarget != target) {
            lastTarget = target;
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(state, 0, 0), new MotionState(target, 0, 0), maxVel, maxAccel,maxJerk);
            timer.reset();
        }
        else{ lastTarget = target; }
        MotionState motionState = profile.get(timer.seconds());
        return motionState.getX();
    }
}
