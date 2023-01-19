package org.firstinspires.ftc.teamcode.subs;

public class linearSlide {

    private final motorGroup motors;
    private final runMotionProfile profile = new runMotionProfile(10000,11000,20000,0.2,0,0,0);

    public linearSlide(motorGroup motors){
        this.motors = motors;
    }

    public void moveTo(double target){
        motors.setPowers(profile.profiledMovement(target,motors.getAveragePosition()));
    }

    public void setMotionConstraints(double maxVel, double maxAccel, double maxJerk){
        profile.setMotionConstraints(maxVel, maxAccel, maxJerk);
    }

    public void setPIDcoeffs(double Kp, double Kd, double Ki, double Kf){ profile.setPIDcoeffs(Kp,Kd,Ki,Kf); }

}
