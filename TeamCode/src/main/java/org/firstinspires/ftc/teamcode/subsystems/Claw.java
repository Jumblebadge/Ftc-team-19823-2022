package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Claw {

    public static final double close = 0.8, open = 0.6;
    ServoImplEx claw;

    public Claw(HardwareMap hardwareMap) {
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        claw.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void moveTo(double target) {
        claw.setPosition(target);
    }

    public void close() {
        claw.setPosition(close);
    }

    public void open() {
        claw.setPosition(open);
    }

    public void toggle(boolean active) {
        if (active) { close(); }
        else { open(); }
    }

    public void PWMrelease() {
        claw.setPwmDisable();
    }

}
