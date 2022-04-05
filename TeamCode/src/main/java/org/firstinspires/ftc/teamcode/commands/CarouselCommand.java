package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BoxSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class CarouselCommand extends SequentialCommandGroupEx {

    public CarouselCommand(CarouselSubsystem carousel, boolean reversed) {
        addCommands(new SequentialCommandGroup(
                run(() -> carousel.spinSlow(reversed)),
                waitMillis(Constants.CarouselConstants.Spin.value.SLOW_TO_FAST_WAIT),
                run(() -> carousel.spinFast(reversed)),
                waitMillis(Constants.CarouselConstants.Spin.value.HOW_LONG_IT_STAYS_FAST),
                run(carousel::stop)
        ));
        addRequirements(carousel);
    }




}
