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
                        new SequentialCommandGroup( // runs if something is detected
                                run(trapdoor::close),
                                waitMillis(400L),
                                run(intake::outtake)

                        ),
                        new SequentialCommandGroup(//  runs if nothing is detected
                                run(trapdoor::open),
                                run(intake::intake)
                        ),
                        sensor::detected) // supplier
        );
        addRequirements(intake, trapdoor, sensor);
    }
}
