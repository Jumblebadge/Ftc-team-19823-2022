package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.util.ElapsedTime;

public class myElapsedTime extends ElapsedTime {

    private volatile long nsPausedTime = nsNow();
    private volatile boolean isPaused = false;

    public long nanoseconds() {
        if (isPaused){
            return nsPausedTime - nsStartTime;
        }
        return (nsNow() - nsStartTime);
    }

    public void pauseTimer(){
        nsPausedTime = nsNow();
        isPaused = true;
    }

    public void resumeTimer(){
        long startTime = nsStartTime;
        nsStartTime = startTime + nsNow() - nsPausedTime;
        isPaused = false;
    }

}
