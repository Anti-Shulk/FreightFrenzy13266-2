package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class ArmSetAndMoveToHeight extends SequentialCommandGroupEx {
    public ArmSetAndMoveToHeight(ArmSubsystem arm, Constants.ArmConstants.Value.Height height) {
        addCommands(new SequentialCommandGroup(
                run(() -> arm.setHeight(height)),
                new ConditionalCommand(run(arm::moveToHeight),/* < if true*/ nothing(), /* < if false*/ arm::isOut) //< boolean supplier
//        addRequirements(arm);
//        addCommands(new SequentialCommandGroup(
//                run(() -> arm.setHeight(Constants.ArmConstants.Value.Height.SHARED)),
//                run(arm::moveToHeight)
        ));
    }
}
