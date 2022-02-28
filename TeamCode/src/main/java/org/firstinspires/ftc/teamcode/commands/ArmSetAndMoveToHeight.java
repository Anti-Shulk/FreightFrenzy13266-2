package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BoxSubsystem;

public class ArmSetAndMoveToHeight extends SequentialCommandGroupEx {
    public ArmSetAndMoveToHeight(ArmSubsystem arm, BoxSubsystem box, Constants.ArmConstants.Value.Height height) {
        addCommands(new SequentialCommandGroup(
                run(() -> arm.setHeight(height)),
                run(() -> box.setHeight(height)),
                new ConditionalCommand(
                        new SequentialCommandGroup(// if true
                                run(arm::moveToHeight),
                                run(box::moveToHeight)
                        ),
                        nothing(), // if false
                        arm::isOut) //< boolean supplier
//        addRequirements(arm);
//        addCommands(new SequentialCommandGroup(
//                run(() -> arm.setHeight(Constants.ArmConstants.Value.Height.SHARED)),
//                run(arm::moveToHeight)
        ));
    }
}
