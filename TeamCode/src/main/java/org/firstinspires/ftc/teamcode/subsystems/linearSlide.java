package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxResetMotorEncoderCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utility.runMotionProfile;

public class linearSlide{

    private final motorGroup motors;
    private double target;
    private final runMotionProfile profile = new runMotionProfile(25000,20000,50000,0.3,0,0,0);

    public linearSlide(HardwareMap hardwareMap){
        DcMotorEx liftLeft = hardwareMap.get(DcMotorEx.class,"Llift");
        DcMotorEx liftRight = hardwareMap.get(DcMotorEx.class,"Rlift");
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new motorGroup(liftLeft,liftRight);
    }

    public void moveTo(double target){
        motors.setPower(profile.profiledMovement(-target, motors.getPosition(0)),0);
        motors.setPower(profile.profiledMovement(-target, motors.getPosition(0)),1);
        this. target = -target;
    }

    public double getError(){
        return target - motors.getPosition(0);
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
