package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.Set;

public class TurretAndArmCommand extends SequentialCommandGroup {
    public TurretAndArmCommand(ArmSubsystem arm, TurretSubsystem turret) {
        addCommands(new InstantCommand(() -> {
            if (arm.isIntake() && turret.isZero()) {
            }
        }, arm, turret));
    }
}
