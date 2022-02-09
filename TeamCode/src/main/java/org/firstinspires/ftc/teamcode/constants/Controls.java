package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.*;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

@Config
public class Controls {
    public static class Driver {
        public static Button  SLOW        = Button.RIGHT_BUMPER;
        public static Button  TURBO       = Button.LEFT_BUMPER;

        // TODO: maybe there is a way to make it so that the trigger can act as a button with a threshold or something
        public static Trigger INTAKE      = Trigger.LEFT_TRIGGER;
        public static Trigger OUTTAKE     = Trigger.RIGHT_TRIGGER;
    }
    public static class Operator {
        public static Button  ARM_FORWARD = Button.DPAD_UP;
        public static Button  ARM_LEFT    = Button.DPAD_LEFT;
        public static Button  ARM_RIGHT   = Button.DPAD_RIGHT;
        public static Button  ARM_INTAKE  = Button.DPAD_DOWN;

        // TODO: maybe there is a way to make it so that the trigger can act as a button with a threshold or something
        public static Trigger ARM_DOWN    = Trigger.LEFT_TRIGGER;
        public static Trigger ARM_UP      = Trigger.LEFT_TRIGGER;

        public static Button CAROUSEL     = Button.X;
    }
}
