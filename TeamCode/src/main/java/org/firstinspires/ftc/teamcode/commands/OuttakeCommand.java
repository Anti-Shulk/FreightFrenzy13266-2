package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.teamcode.subsystems.ColorRangeSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TrapdoorSubsystem;

public class OuttakeCommand extends SequentialCommandGroupEx{
    public OuttakeCommand (IntakeSubsystem intake) {
        addCommands(
                run(intake::outtake),
                waitMillis(1200),
                run(intake::stop)
        );
        addRequirements(intake);
    }
}
