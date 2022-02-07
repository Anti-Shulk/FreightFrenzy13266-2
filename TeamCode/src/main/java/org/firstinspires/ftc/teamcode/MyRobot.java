package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;

public class MyRobot extends Robot {
    public enum OpModeType {
        TELEOP, AUTO
    }

    // the constructor with a specified opmode type
    public MyRobot(OpModeType type) {
        switch (type) {
            case TELEOP: initTele();
            case AUTO: initAuto();
            default: initTele();
        }

    }

    /*
     * Initialize teleop or autonomous, depending on which is used
     */
    public void initTele() {
        // initialize teleop-specific scheduler
    }

    public void initAuto() {
        // initialize auto-specific scheduler
    }
}
