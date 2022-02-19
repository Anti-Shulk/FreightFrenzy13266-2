package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class ArmInQuick extends SequentialCommandGroupEx {
    public ArmInQuick(ArmSubsystem arm, TurretSubsystem turret) {
        addCommands(new SequentialCommandGroup(
                run(turret::moveIn),
                run(arm::moveSoItWontHitSides),
                waitUntil(turret::isIn),
                run(arm::setDropPower),
                run(arm::moveIn),
                waitMillis(500L),
                run(arm::setOff)
        ));
    }
}
