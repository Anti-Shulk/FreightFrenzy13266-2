package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.teamcode.subsystems.ColorRangeSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TrapdoorSubsystem;

public class IntakeCommand extends SequentialCommandGroupEx{
    public IntakeCommand (IntakeSubsystem intake, TrapdoorSubsystem trapdoor, ColorRangeSensorSubsystem sensor) {
        addCommands(new ConditionalCommand(
                        new SequentialCommandGroup( // on true
//                                run(intake::outtake),
                                run(trapdoor::close)
                        ),
                        run(intake::intake), // on false
                        sensor::detected) // supplier
        );
        addRequirements(intake, trapdoor, sensor);
    }
}
