package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.constants.Constants.TrapdoorConstants.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;

public class TrapdoorSubsystem extends HardwareSubsystem {

    private final ServoEx trapdoor;

    public TrapdoorSubsystem() {
        trapdoor = new SimpleServo(hardwareMap, hardware.ID, hardware.MIN_ANGLE, hardware.MAX_ANGLE);
        trapdoor.setInverted(hardware.REVERSED);
        intake();
    }
    public void open() {
        trapdoor.turnToAngle(value.OPEN);
    }
    public void close() {
        trapdoor.turnToAngle(value.CLOSE);
    }
    public void intake() {
        trapdoor.turnToAngle(value.INTAKE);
    }
    public void capHigh() {
        trapdoor.turnToAngle(value.CAP_HIGH);
    }
    public void capLow() {
        trapdoor.turnToAngle(value.CAP_LOW);
    }
    public void capPickUp() {
        trapdoor.turnToAngle(value.CAP_PICKUP);
    }

    @Override
    public void periodic() {

    }
}
