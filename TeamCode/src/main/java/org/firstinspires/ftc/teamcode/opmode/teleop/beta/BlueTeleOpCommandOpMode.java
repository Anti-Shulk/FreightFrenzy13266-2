package org.firstinspires.ftc.teamcode.opmode.teleop.beta;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

@TeleOp(name = "Blue Tele Op Command Op Mode")
public class BlueTeleOpCommandOpMode extends CommandOpMode {
    @Override
    public void initialize() {
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        GamepadEx operatorGamepad = new GamepadEx(gamepad2);

        ArmSubsystem arm = new ArmSubsystem(hardwareMap, "armMotor");

        operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(arm::up);

        waitForStart();
        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
        }
    }
}
