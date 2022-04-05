package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSubsystem;

public class CarouselStopCommand extends SequentialCommandGroupEx {

    public CarouselStopCommand(CarouselSubsystem carousel) {
        addCommands(new SequentialCommandGroup(
                run(carousel::stop)
        ));
        addRequirements(carousel);
    }




}
