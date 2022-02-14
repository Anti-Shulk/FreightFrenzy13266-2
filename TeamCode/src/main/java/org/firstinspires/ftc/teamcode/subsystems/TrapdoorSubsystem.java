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
        trapdoor = new SimpleServo(hardwareMap, hardware.ID, hardware.MIN_ANGLE, hardware.MIN_ANGLE);
        trapdoor.setInverted(false);
    }
    public void open() {
        trapdoor.turnToAngle(value.OPEN);
    }
    public void close() {
        trapdoor.turnToAngle(value.CLOSE);
    }

    @Override
    public void periodic() {

    }
}
