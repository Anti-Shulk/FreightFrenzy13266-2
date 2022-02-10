package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.constants.Constants.TrapdoorConstants.*;

import org.firstinspires.ftc.teamcode.constants.Constants;

public class TrapdoorSubsystem extends SubsystemBase {

    private ServoEx trapdoor;

    public TrapdoorSubsystem(HardwareMap hardwareMap) {
        trapdoor = new SimpleServo(hardwareMap, hardware.ID, hardware.MIN_ANGLE, hardware.MIN_ANGLE);
        trapdoor.setInverted(false);
    }
    public void open() {
        trapdoor.turnToAngle(value.OPEN);
    }
    public void close() {
        trapdoor.turnToAngle(value.CLOSE);
    }
}
