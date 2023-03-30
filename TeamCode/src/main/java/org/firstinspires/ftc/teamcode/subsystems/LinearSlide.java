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
    private final RunMotionProfile profile = new RunMotionProfile(60000,70000,80000,0.1,0,1,0.2, 1);

    public static final double highPole = 800, mediumPole = 400, transfer = 270, autotransfer = 300, zero = 0;
    double currentPole = zero, offset = 0;
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
        motors.setPower(profile.profiledMovement(target, motors.getPosition(0) + offset),0);
        motors.setPower(profile.profiledMovement(target, motors.getPosition(0) + offset),1);
    }

    public double getError(){
        return target - (motors.getPosition(0) + offset);
    }

    public double getPosition() { return motors.getPosition(0) + offset; }

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

    public void setAlignerPosition(double position) {
        aligner.setPosition(position);
    }

    public void PWMrelease() {
        aligner.setPwmDisable();
    }

    public void addOffset(double offset) {
        this.offset = offset;
    }

    public void highPole(){
        moveTo(highPole);
        aligner.setPosition(alignerUp);
        currentPole = highPole;
    }
    public void mediumPole(){
        moveTo(mediumPole);
        aligner.setPosition(alignerUp);
        currentPole = mediumPole;
    }
    public void transfer(){
        moveTo(transfer);
        aligner.setPosition(alignerDown);
        currentPole = transfer;
    }
    public void auto(){
        moveTo(autotransfer);
        aligner.setPosition(alignerDown);
        currentPole = transfer;
    }
    public void zero(boolean override){
        moveTo(zero);
        aligner.setPosition((override) ? alignerDown : alignerUp);
        currentPole = zero;
    }

    public void toggleAligner(boolean bool) {
        aligner.setPosition((bool) ? alignerUp : alignerDown);
    }

    public double returnPole() {
        return currentPole;
    }

}
