package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BoxSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class TurretArmInQuick extends SequentialCommandGroupEx {

    public TurretArmInQuick(ArmSubsystem arm, TurretSubsystem turret, BoxSubsystem box) {

        addCommands(
                new ConditionalCommand(
                    new SequentialCommandGroup(
                            run(arm::moveSoItWontHitSides),
                            waitMillis(500L),
                            run(arm::moveIn)
                    ), // if false
                    new SequentialCommandGroup( // if true
                            run(arm::moveSoItWontHitSides),
                            run(box::moveDown),
//                        waitUntil(arm::isAtTarget),
                            waitMillis(200),
                        run(turret::moveIn),



//                        waitMillis(200),
                        //TODO: maybe create another intake in where the box doesnt move in quite lit this. the reason i had
                            // to make it like this is because the box owuld pull down the hub if it was on high and it was pressed down before it was back

                        waitUntil(turret::isIn),
//                        waitMillis(300L),
//                            run(box::moveDown),
//                        waitMillis(500L), // TODO: this delay is alot and its extra try to get rid of it with another command that has this delay
                        run(arm::setDropPower),
                        run(arm::moveIn),
                        run(arm::setIsIn),
                        waitMillis(500L),
                        run(turret::resetEncoder)
                    ),
                    arm::isIn // the condition
//                waitMillis(500L),
//                run(arm::setOff)
        ));

        addRequirements(arm, turret, box);

    }
    public TurretArmInQuick(ArmSubsystem arm, TurretSubsystem turret, BoxSubsystem box, double waitSeconds) {
        long wait;
        wait = (long) (waitSeconds * 1000);

        addCommands(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                run(arm::moveSoItWontHitSides),
                                waitMillis(500),
                                run(arm::moveIn)
                        ), // if false
                        new SequentialCommandGroup( // if true
                                waitMillis(wait),
                                run(arm::moveSoItWontHitSides),
                                waitMillis(800),
                                run(turret::moveIn),


                                run(box::moveDown),
                                //TODO: maybe create another intake in where the box doesnt move in quite lit this. the reason i had
                                // to make it like this is because the box owuld pull down the hub if it was on high and it was pressed down before it was back

                                waitUntil(turret::isIn),
                                waitMillis(300L),
//                            run(box::moveDown),
//                        waitMillis(500L), // TODO: this delay is alot and its extra try to get rid of it with another command that has this delay
                                run(arm::setDropPower),
                                run(arm::moveIn),
                                run(arm::setIsIn),
                                waitMillis(500L),
                                run(turret::resetEncoder)
                        ),
                        arm::isIn // the condition
//                waitMillis(500L),
//                run(arm::setOff)
                ));

        addRequirements(arm, turret, box);

    }
}
