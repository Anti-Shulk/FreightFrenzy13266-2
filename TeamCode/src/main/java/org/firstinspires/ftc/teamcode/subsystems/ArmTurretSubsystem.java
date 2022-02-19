package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

public class ArmTurretSubsystem extends HardwareSubsystem{

    ArmSubsystem arm;
    TurretSubsystem turret;
    TrapdoorSubsystem trapdoor;
    private boolean flipped = false;
    private boolean positive = true;

    public ArmTurretSubsystem (){
        arm = new ArmSubsystem();
        turret = new TurretSubsystem();
        trapdoor = new TrapdoorSubsystem();
    }

    @Override
    public void init () {
        arm.init();
        turret.init();
        trapdoor.init();
    }

    @Override
    public void periodic() {
        arm.periodic();
        turret.periodic();
        trapdoor.periodic();
    }

    public void moveTo(boolean flipped) {
        arm.moveHigh();
        while (arm.isSus()) {
        }
        turret.moveForward();
    }

    public void moveIn() {

    }
}
