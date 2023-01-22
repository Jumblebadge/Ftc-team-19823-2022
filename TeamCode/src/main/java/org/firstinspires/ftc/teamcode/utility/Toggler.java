package org.firstinspires.ftc.teamcode.utility;

public class Toggler {

    private boolean current = false;
    private boolean last = false;

    public boolean update(boolean currentState) {
        if (currentState && !last) {
            current = !current;
        }

        last = currentState;
        return current;
    }
}
