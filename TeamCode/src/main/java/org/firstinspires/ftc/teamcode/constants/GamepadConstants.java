package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.*;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

@Config
public class GamepadConstants {
    public static Control button = new Control();
    public static Value value = new Value();
    // TODO: Make it so that you can change which driver has which button
    // and maybe make it so the buttons are configurable in FTC Dashboard

    // TODO: maybe make it so that in FTC dasshboard, you can activate any of the functions in
    // any of the subsystems somehow, or maybe ill make it myself
    // idk
    public static class Control {

        public Button  SLOW        = Button.RIGHT_BUMPER;
        public Button  TURBO       = Button.LEFT_BUMPER;

        public Button DROP = Button.LEFT_BUMPER; // this is the same button as turbo mode, so there might not be a turbo mode
        public Button  CLOSE = Button.RIGHT_BUMPER;

        // TODO: maybe there is a way to make it so that the trigger can act as a button with a threshold or something
        public Trigger INTAKE      = Trigger.RIGHT_TRIGGER;
        public Trigger OUTTAKE     = Trigger.LEFT_TRIGGER;
        public Button  FORWARD = Button.DPAD_UP;
        public Button  LEFT    = Button.DPAD_LEFT;
        public Button  RIGHT   = Button.DPAD_RIGHT;
        public Button  DOWN = Button.DPAD_DOWN;
        public Button  SHARED_BOX_UP = Button.Y;


        // TODO: maybe there is a way to make it so that the trigger can act as a button with a threshold or something
        public Trigger LEFT_SHARED = Trigger.LEFT_TRIGGER;
        public Trigger RIGHT_SHARED = Trigger.RIGHT_TRIGGER;

        public Button CAROUSEL_LIFT = Button.A;
        public Button CAROUSEL_RED  = Button.B;
        public Button CAROUSEL_BLUE  = Button.X;

        public Button GRIPPER_LIFT  = Button.A;
        public Button GRIPPER_GRAB = Button.B;
        public Button TOGGLE_INTAKE_UP  = Button.Y;

        public Button RESET_IMU  = Button.A;





    }
    public static class Value {
        public double STICK_THRESHOLD = 0;
        public double STICK_TOUCHING_THRESHOLD = 0.3;
        public double TRIGGER_THRESHOLD = 0.3;
    }
}
