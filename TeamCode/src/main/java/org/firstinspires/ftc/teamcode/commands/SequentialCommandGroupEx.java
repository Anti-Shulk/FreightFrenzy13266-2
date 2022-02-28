package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import java.util.function.BooleanSupplier;

public class SequentialCommandGroupEx extends SequentialCommandGroup {
    public InstantCommand run(Runnable runnable) {
        return new InstantCommand(runnable);
    }
    public InstantCommand nothing() {
        return new InstantCommand();
    }

    public WaitUntilCommand waitUntil(BooleanSupplier booleanSupplier) {
        return new WaitUntilCommand(booleanSupplier);
    }
    public WaitCommand waitMillis(Long millis) {
        return new WaitCommand(millis);
    }
    public WaitCommand waitMillis(int millis) {
        return new WaitCommand(millis);
    }
}
