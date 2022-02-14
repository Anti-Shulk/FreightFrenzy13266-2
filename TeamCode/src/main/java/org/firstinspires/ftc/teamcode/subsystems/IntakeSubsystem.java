package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.Constants.IntakeConstants.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.MotorExEx;

public class IntakeSubsystem extends HardwareSubsystem {
    private final MotorExEx intake;

    public IntakeSubsystem() {
        intake = new MotorExEx(hardwareMap, hardware.ID, hardware.CPR, hardware.RPM);
        intake.setInverted(hardware.REVERSED);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intake.setRunMode(Motor.RunMode.VelocityControl);
        intake.set(controller.INIT_POWER);

    }

    @Override
    public void periodic() {

    }


    public void intake() {
        intake.set(controller.POWER);
    }
    public void outtake() {
        intake.set(-controller.POWER);
    }
    public void stop() {
        intake.set(0);
    }
}
