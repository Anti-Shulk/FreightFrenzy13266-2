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
                new WaitUntilCommand(arm::wontHitSides),
                new InstantCommand(() -> turret.setTargetDegrees(turretPos))
         ));
    }

    public ArmOutQuick(ArmSubsystem arm, TurretSubsystem turret, double turretPos) {
        addCommands(new SequentialCommandGroup(
                new InstantCommand(arm::moveToTarget),
                new WaitUntilCommand(arm::wontHitSides),
                new InstantCommand(() -> turret.setTargetDegrees(turretPos))
        ));
    }

}
