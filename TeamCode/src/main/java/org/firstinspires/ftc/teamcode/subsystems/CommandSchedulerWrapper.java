package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.function.BooleanSupplier;

public class CommandSchedulerWrapper {
    private OpMode opMode;
    public CommandSchedulerWrapper (OpMode opMode) {
        this.opMode = opMode;
    }

    public Button add(BooleanSupplier button) {
        return new BooleanButton(button);
    }

    public void addDefault(Runnable toRun) {
        CommandScheduler.getInstance().addButton(toRun);
    }
}
