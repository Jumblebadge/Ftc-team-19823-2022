package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Linkage {

    public static final double out = 0.45, in = 0.15, auto = 0.51;
    ServoImplEx linkage;

    public Linkage(HardwareMap hardwareMap) {
        linkage = hardwareMap.get(ServoImplEx.class, "linkage");
        linkage.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void moveTo(double target) {
        linkage.setPosition(target);
    }

    public void out() {
        linkage.setPosition(out);
    }

    public void in() {
        linkage.setPosition(in);
    }

    public void auto() {
        linkage.setPosition(auto);
    }

}
