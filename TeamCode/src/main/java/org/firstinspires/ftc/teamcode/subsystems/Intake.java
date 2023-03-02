package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Intake {

    public static final double cone = 0.975, vertical = 0.45, transfer = 0.3, init = 0.3, lowJunction = 0.7, beacon = 1;
    TwoServo intake;

    public Intake(HardwareMap hardwareMap) {
        ServoImplEx intakeFlipL = hardwareMap.get(ServoImplEx.class, "inL");
        ServoImplEx intakeFlipR = hardwareMap.get(ServoImplEx.class, "inR");

        intakeFlipL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeFlipR.setPwmRange(new PwmControl.PwmRange(500, 2500));

        intake = new TwoServo(intakeFlipL, intakeFlipR);
    }

    public void moveTo(double target) {
        intake.moveTo(target);
    }

    public void cone() {
        intake.moveTo(cone);
    }

    public void transfer() {
        intake.moveTo(transfer);
    }

    public void init() {
        intake.moveTo(init);
    }

    public void vertical() {
        intake.moveTo(vertical);
    }

    public void lowJunction() {
        intake.moveTo(lowJunction);
    }

    public void beacon() {
        intake.moveTo(beacon);
    }
}
