package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utility.runMotionProfile;

public class linearSlide{

    private final motorGroup motors;
    private final runMotionProfile profile = new runMotionProfile(25000,20000,50000,0.3,0,0,0);

    public linearSlide(DcMotorEx motor1, DcMotorEx motor2){
        motors = new motorGroup(motor1,motor2);
    }

    public void moveTo(double target){
        motors.setPower(profile.profiledMovement(target, motors.getPosition(0)),0);
        motors.setPower(profile.profiledMovement(target,motors.getPosition(1)),1);
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

    public void resetEncoders(){
        motors.motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.motors[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors.motors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void highPole(){
        moveTo(1100);
    }

}
