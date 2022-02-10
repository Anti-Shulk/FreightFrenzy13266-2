package org.firstinspires.ftc.teamcode.constants;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class GamepadTrigger extends Button {
    private GamepadEx gamepad;
    private GamepadKeys.Trigger trigger;

    public GamepadTrigger(GamepadEx gamepad, GamepadKeys.Trigger trigger) {
        this.gamepad = gamepad;
        this.trigger = trigger;
    }

    @Override
    public boolean get() {
        return gamepad.getTrigger(trigger) > 0.5;
    }
}
