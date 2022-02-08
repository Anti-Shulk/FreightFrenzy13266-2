package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;

public class RobotContainer {

    private final CommandScheduler scheduler;

    // Subsystems
    public RobotContainer() {
        scheduler = CommandScheduler.getInstance();
    }
}
