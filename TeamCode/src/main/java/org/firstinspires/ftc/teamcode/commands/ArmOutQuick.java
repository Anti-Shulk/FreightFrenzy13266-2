package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;


import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.function.BooleanSupplier;

public class ArmOutQuick extends SequentialCommandGroupEx {
//    public ArmOutQuick(ArmSubsystem arm, TurretSubsystem turret, double turretPos, double armPos) {
//        addCommands(new SequentialCommandGroup(
//                new InstantCommand(() -> arm.setDegrees(armPos)),
//                new WaitUntilCommand(arm::isSus),
//                new InstantCommand(() -> turret.setDegrees(turretPos))
//         ));
//    }

    public ArmOutQuick(ArmSubsystem arm, TurretSubsystem turret, Runnable armPos, Runnable turretPos) {
        addCommands(new SequentialCommandGroup(
                run(arm::setTeleOpPower),
                run(armPos),
                waitUntil(arm::isNotSus),
                run(turretPos)
        ));
    }
}
