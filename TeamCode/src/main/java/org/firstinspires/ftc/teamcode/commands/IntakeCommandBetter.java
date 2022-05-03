package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ColorRangeSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TrapdoorSubsystem;

public class IntakeCommandBetter extends CommandBase {
    private final IntakeSubsystem intake;
    private final TrapdoorSubsystem trapdoor;
    private final ColorRangeSensorSubsystem sensor;
    private boolean hasDetected = false;

    public IntakeCommandBetter(IntakeSubsystem intake, TrapdoorSubsystem trapdoor, ColorRangeSensorSubsystem sensor) {
        this.intake = intake;
        this.trapdoor = trapdoor;
        this.sensor = sensor;
        addRequirements(intake, trapdoor, sensor);
    }

    @Override
    public void execute() {
        if (sensor.detected() || hasDetected) {
            trapdoor.close();
            intake.outtake();
            hasDetected = true;
        } else {
            trapdoor.intake();
            intake.intake();
        }
    }

    @Override
    public void end(boolean interrupted) {
        trapdoor.close();
        intake.stop();
    }


}
