package org.firstinspires.ftc.teamcode.subsystems;

public class ArmTurretSubsystem extends HardwareSubsystem {

    ArmSubsystem arm;
    TurretSubsystem turret;

    public ArmTurretSubsystem (){
        arm = new ArmSubsystem();
        turret = new TurretSubsystem();

    }
    @Override
    public void init (){
        arm.init();
        turret.init();

    }
}
