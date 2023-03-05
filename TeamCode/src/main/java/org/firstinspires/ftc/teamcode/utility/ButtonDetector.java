package org.firstinspires.ftc.teamcode.utility;

public class ButtonDetector {

    public boolean current = false;
    public boolean last = false;

    public boolean constantUpdate(boolean currentState) {
        if (currentState && !last) {
            current = !current;
        }

        last = currentState;
        return current;
    }

    public boolean isolatedUpdate(boolean currentState) {
        boolean state = currentState && !last;
        last = currentState;
        return state;
    }

    public void toFalse() {
        current = false;
    }
}
