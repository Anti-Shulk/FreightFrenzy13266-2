package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class TurretArmCommand extends CommandBase {

    protected double multiplier;

    public TurretArmCommand(TurretSubsystem turret, ArmSubsystem arm, GamepadEx gamepad) {


        this.multiplier = 1;
        addRequirements(turret, arm);
    }

    @Override
    public  void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }
}
