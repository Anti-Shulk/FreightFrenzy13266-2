package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BoxSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class CarouselCommand extends SequentialCommandGroupEx {

    public CarouselCommand(CarouselSubsystem carousel, boolean reversed) {
        addCommands(new SequentialCommandGroup(
                run(() -> carousel.spinSlow(reversed)),
                waitMillis(500),
                run(() -> carousel.spinFast(reversed)),
                waitMillis(700),
                run(carousel::stop)
        ));
        addRequirements(carousel);
    }




}
