package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class ArmInQuick extends SequentialCommandGroup {
    public ArmInQuick(ArmSubsystem arm, TurretSubsystem turret) {
        addCommands(new SequentialCommandGroup(
                new InstantCommand(() -> turret.setTargetDegrees(Constants.TurretConstants.value.INITIAL_POSITION)),
                new InstantCommand(arm::moveWontHitSides),
                new WaitUntilCommand(turret::isAtTarget),
                new InstantCommand(arm::moveIntake)
        ));
    }
}
