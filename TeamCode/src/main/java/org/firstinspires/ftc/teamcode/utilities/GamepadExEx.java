package org.firstinspires.ftc.teamcode.utilities;

import static org.firstinspires.ftc.teamcode.constants.GamepadConstants.*;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.constants.GamepadConstants;

public class GamepadExEx extends GamepadEx {
    public GamepadExEx(Gamepad gamepad) {
        super(gamepad);
    }
//
//    @Override
//    public double getLeftX() {
//        return gamepad.left_stick_x >= value.STICK_THRESHOLD ? gamepad.left_stick_x : 0;
//    }
//
//    @Override
//    public double getLeftY() {
//        return -gamepad.left_stick_y >= value.STICK_THRESHOLD ? gamepad.left_stick_y : 0;
//    }
//
//    @Override
//    public double getRightX() {
//        return gamepad.right_stick_x >= value.STICK_THRESHOLD ? gamepad.right_stick_x : 0;
//    }
//
//    @Override
//    public double getRightY() {
//        return -gamepad.right_stick_y >= value.STICK_THRESHOLD ? gamepad.right_stick_y : 0;
//    }

    public boolean getTriggerPressed(GamepadKeys.Trigger trigger) {
        return getTrigger(trigger) > value.TRIGGER_THRESHOLD;
    }

    public boolean getLeftTouchingEdge() {
        return (Math.sqrt((Math.pow(getLeftX(), 2)) + (Math.pow(getLeftY(), 2)))) >  value.STICK_TOUCHING_THRESHOLD;
    }

    public boolean getRightTouchingEdge() {
        return (Math.sqrt((Math.pow(getRightX(), 2)) + (Math.pow(getRightY(), 2)))) >  value.STICK_TOUCHING_THRESHOLD;
    }

    public boolean get(GamepadKeys.Button button) {
        return getButton(button);
    }

    public boolean get(GamepadKeys.Trigger trigger) {
        return getTrigger(trigger) >  value.TRIGGER_THRESHOLD;
    }

}






