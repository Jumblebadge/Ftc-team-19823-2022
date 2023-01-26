package org.firstinspires.ftc.teamcode.utility;

public class Toggler {

    public boolean current = false;
    public boolean last = false;

    public boolean update(boolean currentState) {
        if (currentState && !last) {
            current = !current;
        }

        last = currentState;
        return current;
    }
}
