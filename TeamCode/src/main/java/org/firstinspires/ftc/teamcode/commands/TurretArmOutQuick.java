package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;


import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BoxSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.function.BooleanSupplier;

public class TurretArmOutQuick extends SequentialCommandGroupEx {


//    public ArmOutQuick(ArmSubsystem arm, TurretSubsystem turret, double turretPos, double armPos) {
//        addCommands(new SequentialCommandGroup(
//                new InstantCommand(() -> arm.setDegrees(armPos)),
//                new WaitUntilCommand(arm::isSus),
//                new InstantCommand(() -> turret.setDegrees(turretPos))
////         ));
////    }
//
//    public TurretArmOutQuick(ArmSubsystem arm, TurretSubsystem turret, BoxSubsystem box, Runnable turretPos, Runnable armPos) {
//        addCommands(new SequentialCommandGroup(
//                run(arm::setTeleOpPower),
//                run(armPos),
//                waitUntil(arm::isNotSus),
//                run(turretPos)
//        ));
//
//        this.arm = arm;
//        this.turret = turret;
//        this.box = box;
//        addRequirements(arm, turret, box);
//    }

    public TurretArmOutQuick(ArmSubsystem arm, TurretSubsystem turret, BoxSubsystem box, Runnable turretPos) {
        addCommands(new SequentialCommandGroup(
                new ConditionalCommand(
                        nothing(), // if true
                        new SequentialCommandGroup( // if false
                            run(arm::moveToNonSusHeight),
                            waitUntil(arm::isNotSus),
                            waitMillis(100L)),
                        arm::isOut), // boolena supplier
//                run(arm::setTeleOpPower),
//                run(arm::moveToHeight),


                run(box::moveHigh),
                run(turretPos),
                new ConditionalCommand(nothing(), // if false
                        new SequentialCommandGroup(// if true
                                waitUntil(turret::isAtTarget),
                                run(arm::moveToHeight)),
                        arm::isOut), // boolean supplier
                run(arm::setIsOut)


        ));

        addRequirements(arm, turret, box);
    }
    public TurretArmOutQuick(ArmSubsystem arm, TurretSubsystem turret, BoxSubsystem box, Runnable turretPos, boolean delayed) {
        long wait = 0;
        if (delayed) wait = 800;
        addCommands(new SequentialCommandGroup(
                waitMillis(wait),
                new ConditionalCommand(
                        nothing(), // if true
                        new SequentialCommandGroup( // if false
                                run(arm::moveToNonSusHeight),
                                waitUntil(arm::isNotSus),
                                waitMillis(100L)),
                        arm::isOut), // boolena supplier
//                run(arm::setTeleOpPower),
//                run(arm::moveToHeight),


                run(box::moveHigh),
                run(turretPos),
                new ConditionalCommand(nothing(), // if false
                        new SequentialCommandGroup(// if true
                                waitUntil(turret::isAtTarget),
                                run(arm::moveToHeight)),
                        arm::isOut), // boolean supplier
                run(arm::setIsOut)


        ));

        addRequirements(arm, turret, box);
    }




}
