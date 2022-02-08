package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

@Config
public class Controls {
    public static class Driver {
        public static GamepadKeys.Button  SLOW        = GamepadKeys.Button.RIGHT_BUMPER;
        public static GamepadKeys.Button  TURBO       = GamepadKeys.Button.LEFT_BUMPER;

        // TODO: maybe there is a way to make it so that the trigger can act as a button with a threshold or something
        public static GamepadKeys.Trigger INTAKE      = GamepadKeys.Trigger.LEFT_TRIGGER;
        public static GamepadKeys.Trigger OUTTAKE     = GamepadKeys.Trigger.RIGHT_TRIGGER;
    }
    public static class Operator {
        public static GamepadKeys.Button  ARM_FORWARD = GamepadKeys.Button.DPAD_UP;
        public static GamepadKeys.Button  ARM_LEFT    = GamepadKeys.Button.DPAD_LEFT;
        public static GamepadKeys.Button  ARM_RIGHT   = GamepadKeys.Button.DPAD_RIGHT;
        public static GamepadKeys.Button  ARM_INTAKE  = GamepadKeys.Button.DPAD_RIGHT;

        // TODO: maybe there is a way to make it so that the trigger can act as a button with a threshold or something
        public static GamepadKeys.Trigger ARM_DOWN    = GamepadKeys.Trigger.LEFT_TRIGGER;
        public static GamepadKeys.Trigger ARM_UP      = GamepadKeys.Trigger.LEFT_TRIGGER;
    }
}
