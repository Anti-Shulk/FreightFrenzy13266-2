package org.firstinspires.ftc.teamcode.opmode.teleop.beta;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;


@TeleOp(name = "Blue Tele Op Command Op Mode")
public class BlueTeleOpCommandOpMode extends CommandOpMode {
    @Override
    public void initialize() {
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        GamepadEx operatorGamepad = new GamepadEx(gamepad2);

//        ArmSubsystem arm = new ArmSubsystem(hardwareMap);
//        CarouselSubsystem carousel = new CarouselSubsystem(hardwareMap);
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(hardwareMap);

//        operatorGamepad.getGamepadButton(Operator.ARM_INTAKE).whenPressed(arm::moveHigh);
//        operatorGamepad.getGamepadButton(Operator.ARM_RIGHT).whenPressed(arm::moveLow);
//        operatorGamepad.getGamepadButton(Operator.ARM_RIGHT).whenPressed(arm::moveLow);
////        Button hi = () -> {
////            if (operatorGamepad.getTrigger(Operator.ARM_UP) > 0.3 ) {
////                arm.moveHigh();
////            } else {
////                arm.moveLow();
////            }
////        };
//        BooleanSupplier qwer = () -> operatorGamepad.getTrigger(Operator.ARM_UP) > 0.3;
////        Button hi = new Button(qwer);
//        Button hi = new GamepadTrigger(operatorGamepad, Operator.ARM_UP).whenPressed(arm::moveHigh);
////        CommandScheduler.getInstance().addButton(arm::moveHigh);
//        // how to put it on another thread
////        CommandScheduler.getInstance().addButton(() -> new Thread(arm::moveHigh).start());
//
////        operatorGamepad.getGamepadButton(Operator.CAROUSEL).whenPressed(carousel::spinForward);

//        Button armUp = new GamepadTrigger(operatorGamepad, Operator.ARM_UP).whenPressed(arm::moveHigh);
        waitForStart();
        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
        }
    }
}
