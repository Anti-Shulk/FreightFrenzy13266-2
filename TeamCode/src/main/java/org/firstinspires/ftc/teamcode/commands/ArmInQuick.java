package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class ArmInQuick extends SequentialCommandGroup {
    public ArmInQuick(ArmSubsystem arm, TurretSubsystem turret) {
        addCommands(new SequentialCommandGroup(
                turret::moveIn,
                arm::moveUpSoItWontHitSides,
                new WaitUntilCommand(turret::isIn),
                arm::moveIn,
                arm::setDropPower,
                new WaitCommand(500),
                arm::setOff
        ));
    }
}
