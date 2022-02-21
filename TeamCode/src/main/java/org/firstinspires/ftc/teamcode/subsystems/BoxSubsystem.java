package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import static org.firstinspires.ftc.teamcode.constants.Constants.BoxConstants.*;

public class BoxSubsystem extends HardwareSubsystem{
    ServoEx boxServo;
    public BoxSubsystem() {
        boxServo = new SimpleServo(hardwareMap, hardware.ID, hardware.MIN_ANGLE, hardware.MAX_ANGLE);
        boxServo.turnToAngle(value.DOWN);
    }

    public void moveHigh() {
        boxServo.turnToAngle(value.HIGH);
    }

    public void moveDown() {
        boxServo.turnToAngle(value.DOWN);
    }

    public void moveMid() {
        boxServo.turnToAngle(value.MID);
    }

    public void moveLow() {
        boxServo.turnToAngle(value.LOW);
    }
}
