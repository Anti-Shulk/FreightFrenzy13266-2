package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class TurretArmCommand extends CommandBase {
    private TurretSubsystem turret;
    private ArmSubsystem arm;
    private GamepadEx gamepad;

    protected double multiplier;

    public TurretArmCommand(TurretSubsystem turret, ArmSubsystem arm, GamepadEx gamepad) {
        this.gamepad = gamepad;

        this.turret = turret;
        this.arm = arm;


        this.multiplier = 1;
        addRequirements(this.turret, this.arm);
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
