package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BoxSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class TurretArmInQuick extends SequentialCommandGroupEx {

    public TurretArmInQuick(ArmSubsystem arm, TurretSubsystem turret, BoxSubsystem box) {

        addCommands(
                new ConditionalCommand(nothing(), // if false
                        new SequentialCommandGroup( // if true
                        run(turret::moveIn),

                        run(arm::moveSoItWontHitSides),
                        run(box::moveDown),
                        waitUntil(turret::isIn),
                        waitMillis(100L),
                        run(arm::setDropPower),
                        run(arm::moveIn),
                        run(arm::setIsIn)), arm::isIn // the condition
//                waitMillis(500L),
//                run(arm::setOff)
        ));

        addRequirements(arm, turret, box);

    }
}
