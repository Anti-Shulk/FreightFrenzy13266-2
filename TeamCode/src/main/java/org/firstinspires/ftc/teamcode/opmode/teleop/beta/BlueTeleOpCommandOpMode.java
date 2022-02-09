package org.firstinspires.ftc.teamcode.opmode.teleop.beta;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.constants.Controls.*;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

@TeleOp(name = "Blue Tele Op Command Op Mode")
public class BlueTeleOpCommandOpMode extends CommandOpMode {
    @Override
    public void initialize() {
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        GamepadEx operatorGamepad = new GamepadEx(gamepad2);

        ArmSubsystem arm = new ArmSubsystem(hardwareMap);
//        CarouselSubsystem carousel = new CarouselSubsystem(hardwareMap);

        operatorGamepad.getGamepadButton(Operator.ARM_INTAKE).whenPressed(arm::moveHigh);
        operatorGamepad.getGamepadButton(Operator.ARM_RIGHT).whenPressed(arm::moveLow);


//        operatorGamepad.getGamepadButton(Operator.CAROUSEL).whenPressed(carousel::spinForward);

        waitForStart();
        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
        }
    }
}
