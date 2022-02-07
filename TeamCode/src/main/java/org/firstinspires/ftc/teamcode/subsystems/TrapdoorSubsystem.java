package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TrapdoorSubsystem extends SubsystemBase {

    private ServoEx trapdoor;
    TrapdoorSubsystem(final HardwareMap hMap, final String name) {
        ServoEx trapdoor = new SimpleServo(hMap, name, 0, 270);
        trapdoor.setInverted(false);
    }
    public void open() {
        trapdoor.turnToAngle(0);
    }
    public void close() {
        trapdoor.turnToAngle(270);
    }
}
