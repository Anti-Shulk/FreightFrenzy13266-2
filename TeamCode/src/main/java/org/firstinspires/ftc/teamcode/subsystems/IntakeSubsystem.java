package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.Constants.IntakeConstants.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utilities.MotorExEx;

public class IntakeSubsystem extends SubsystemBase {
    private final MotorExEx intake;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake = new MotorExEx(hardwareMap, hardware.ID, hardware.CPR, hardware.RPM);
        intake.setInverted(hardware.REVERSED);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intake.setRunMode(Motor.RunMode.VelocityControl);
        intake.set(controller.INIT_POWER);

    }


    public void intake() {
        intake.set(controller.POWER);
    }
    public void outtake() {
        intake.set(-controller.POWER);
    }
}
