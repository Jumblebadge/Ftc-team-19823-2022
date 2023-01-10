package org.firstinspires.ftc.teamcode.subs;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.maths.controlLoopMath;

public class runMotorMotionProfile {

    private final DcMotorEx motor;
    private double lastTarget;
    private double maxVel,maxAccel,maxJerk;
    private final ElapsedTime timer = new ElapsedTime();
    private final controlLoopMath PID = new controlLoopMath(0,0,0,0);
    private MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0,0,0),new MotionState(1,0,0),1,1,1);

    public runMotorMotionProfile(DcMotorEx motor){ this.motor=motor; }

    public void setMotionConstraints(double maxVel, double maxAccel, double maxJerk){
        this.maxVel=maxVel;
        this.maxAccel=maxAccel;
        this.maxJerk=maxJerk;
    }

    public void setPIDcoeffs(double Kp, double Kd, double Ki){ PID.setPIDCoeffs(Kp,Kd,Ki,0); }

    public void runProfile(double target){
        if (lastTarget != target) {
            lastTarget = target;
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(motor.getCurrentPosition(), 0, 0), new MotionState(target, 0, 0), maxVel, maxAccel,maxJerk);
            timer.reset();
        }
        else{ lastTarget = target; }
        MotionState motionState = profile.get(timer.seconds());
        motor.setPower(PID.PIDout(motionState.getX()-motor.getCurrentPosition()));
    }
}
