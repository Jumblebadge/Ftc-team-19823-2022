package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.utility.RunMotionProfile;

public class LinearSlide {

    private final MotorGroup motors;
    private double target;
    private final ServoImplEx aligner;
    private final RunMotionProfile profile = new RunMotionProfile(25000,20000,50000,0.1,0,1,0.2, 1);

    final double highPole = 1025, mediumPole = 500, transfer = 250, zero = 0;
    final double alignerDown = 0.75, alignerUp = 1;

    public LinearSlide(HardwareMap hardwareMap){
        DcMotorEx liftLeft = hardwareMap.get(DcMotorEx.class,"Llift");
        DcMotorEx liftRight = hardwareMap.get(DcMotorEx.class,"Rlift");
        aligner = hardwareMap.get(ServoImplEx.class, "aligner");

        aligner.setPwmRange(new PwmControl.PwmRange(500, 2500));
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new MotorGroup(liftLeft,liftRight);
    }

    public void moveTo(double target){
        this.target = -target;
    }

    public void update() {
        motors.setPower(profile.profiledMovement(target, motors.getPosition(0)),0);
        motors.setPower(profile.profiledMovement(target, motors.getPosition(0)),1);
    }

    public double getError(){
        return target - motors.getPosition(0);
    }

    public double getPosition() { return motors.getPosition(0); }

    public boolean isTimeDone() { return profile.getProfileDuration() < profile.getCurrentTime(); }
    public boolean isPositionDone() { return Math.abs(getError()) < 10; }

    public void setMotionConstraints(double maxVel, double maxAccel, double maxJerk){
        profile.setMotionConstraints(maxVel, maxAccel, maxJerk);
    }

    public void setPIDcoeffs(double Kp, double Kd, double Ki, double Kf, double limit){
        profile.setPIDcoeffs(Kp, Kd, Ki, Kf, limit);
    }

    public double getMotionTarget(){
        return -profile.getMotionTarget();
    }

    public double getTarget() { return -target; }

    public double getMotionTime() { return profile.getCurrentTime(); }

    public void resetEncoders(){
        motors.motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.motors[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors.motors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void highPole(){
        moveTo(highPole);
        aligner.setPosition(alignerUp);
    }
    public void mediumPole(){
        moveTo(mediumPole);
        aligner.setPosition(alignerUp);
    }
    public void transfer(){
        moveTo(transfer);
        aligner.setPosition(alignerDown);
    }
    public void zero(){
        moveTo(zero);
        aligner.setPosition(alignerUp);
    }

}