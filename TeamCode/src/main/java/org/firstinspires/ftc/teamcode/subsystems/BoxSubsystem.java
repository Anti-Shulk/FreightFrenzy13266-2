package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;


import static org.firstinspires.ftc.teamcode.constants.Constants.ArmConstants.Value.Height.*;
import static org.firstinspires.ftc.teamcode.constants.Constants.BoxConstants.*;

import org.firstinspires.ftc.teamcode.constants.Constants;

public class BoxSubsystem extends HardwareSubsystem{
    ServoEx boxServo;
    private Constants.ArmConstants.Value.Height height = HIGH;
    public BoxSubsystem() {
        boxServo = new SimpleServo(hardwareMap, hardware.ID, hardware.MIN_ANGLE, hardware.MAX_ANGLE);
        boxServo.turnToAngle(value.DOWN);
        boxServo.setInverted(hardware.REVERSED);
    }

    public void moveToHeight() {
        switch (height) {
            case LOW: moveLow(); break;
            case MID: moveMid(); break;
            case HIGH: moveHigh(); break;
            case AUTO_HIGH: moveHigh(); break;
            case AUTO_MID: moveMid(); break;
            case AUTO_LOW: moveLow(); break;
            case SHARED: moveShared(); break;
            case SHARED_UP: moveSharedUp(); break;
        }
    }
    public void setHeight(Constants.ArmConstants.Value.Height height) {
        this.height = height;
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

    public void moveShared() {
        boxServo.turnToAngle(value.SHARED);
    }

    public void moveSharedUp() {
        boxServo.turnToAngle(value.SHARED_UP);
    }
}
