package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Deposit {

    public static final double score = 0.3, init = 0.175, transfer = 0.8;
    TwoServo deposit;

    public Deposit(HardwareMap hardwareMap) {
        ServoImplEx depositFlipL = hardwareMap.get(ServoImplEx.class, "outL");
        ServoImplEx depositFlipR = hardwareMap.get(ServoImplEx.class, "outR");

        depositFlipL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        depositFlipR.setPwmRange(new PwmControl.PwmRange(500, 2500));

        deposit = new TwoServo(depositFlipL, depositFlipR);
    }

    public void moveTo(double target) {
        deposit.moveTo(target);
    }

    public void score() {
        deposit.moveTo(score);
    }

    public void transfer() {
        deposit.moveTo(transfer);
    }

    public void init() {
        deposit.moveTo(init);
    }

}
