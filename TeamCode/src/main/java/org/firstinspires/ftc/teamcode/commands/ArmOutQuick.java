package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class ArmOutQuick extends SequentialCommandGroup {
    public ArmOutQuick(ArmSubsystem arm, TurretSubsystem turret, double turretPos, double armPos) {
        addCommands(new SequentialCommandGroup(
                new InstantCommand(() -> arm.setDegrees(armPos)),
                new WaitUntilCommand(arm::isSus),
                new InstantCommand(() -> turret.setDegrees(turretPos))
         ));
    }

    public ArmOutQuick(ArmSubsystem arm, TurretSubsystem turret) {
        addCommands(new SequentialCommandGroup(
                new InstantCommand(arm::setTeleOpPower),
                new InstantCommand(arm::moveToTarget),
                new WaitUntilCommand(arm::isSus),
                new InstantCommand(turret::moveToTargetDegrees
        )));
    }

}
