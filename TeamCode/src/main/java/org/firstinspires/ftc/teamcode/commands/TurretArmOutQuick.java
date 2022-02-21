package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;


import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BoxSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.function.BooleanSupplier;

public class TurretArmOutQuick extends SequentialCommandGroupEx {
    private final ArmSubsystem arm;
    private final TurretSubsystem turret;
    private final BoxSubsystem box;
    private Runnable turretPos;


//    public ArmOutQuick(ArmSubsystem arm, TurretSubsystem turret, double turretPos, double armPos) {
//        addCommands(new SequentialCommandGroup(
//                new InstantCommand(() -> arm.setDegrees(armPos)),
//                new WaitUntilCommand(arm::isSus),
//                new InstantCommand(() -> turret.setDegrees(turretPos))
//         ));
//    }

    public TurretArmOutQuick(ArmSubsystem arm, TurretSubsystem turret, BoxSubsystem box, Runnable turretPos, Runnable armPos) {
        addCommands(new SequentialCommandGroup(
                run(arm::setTeleOpPower),
                run(armPos),
                waitUntil(arm::isNotSus),
                run(turretPos)
        ));

        this.arm = arm;
        this.turret = turret;
        this.box = box;
        addRequirements(arm, turret, box);
    }

    public TurretArmOutQuick(ArmSubsystem arm, TurretSubsystem turret, BoxSubsystem box, Runnable turretPos) {
        addCommands(new SequentialCommandGroup(
                new ConditionalCommand(nothing(), run(arm::moveToNonSusHeight), arm::isOut),
//                run(arm::setTeleOpPower),
//                run(arm::moveToHeight),
                waitUntil(arm::isNotSus),
                waitMillis(100L),
                run(box::moveHigh),
                run(turretPos),
                run(arm::moveToHeight),
                run(arm::setIsOut)


        ));

        this.arm = arm;
        this.turret = turret;
        this.box = box;
        this.turretPos = turretPos;
        addRequirements(arm, turret, box);
    }




}
