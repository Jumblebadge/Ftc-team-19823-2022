package org.firstinspires.ftc.teamcode.maths;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class motionProfile {

    static MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0,0,0),new MotionState(1,0,0),1,1);

    public static MotionState RRprofile(double target, double lasttarget, ElapsedTime time, double pos){

        if (lasttarget != target) {
            lasttarget = target;
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(pos, 0, 0), new MotionState(target, 0, 0), 2900, 2900);
            time.reset();
        }
        else{ lasttarget = target; }

        //TODO make it output lasttarget or else its broken

        return profile.get(time.seconds());
    }
}
