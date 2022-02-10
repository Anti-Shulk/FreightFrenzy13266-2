package org.firstinspires.ftc.teamcode.opmode.teleop.beta;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;


@TeleOp(name = "Blue Tele Op Command Op Mode")
public class BlueTeleOpCommandOpMode extends CommandOpMode {
    @Override
    public void initialize() {
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx operator = new GamepadEx(gamepad2);

        ArmSubsystem arm = new ArmSubsystem(hardwareMap);
//        CarouselSubsystem carousel = new CarouselSubsystem(hardwareMap);
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(hardwareMap);

        // a button is a runnable method.

        // In this command right here, we are creating running the getgamepadbutton constructor
        // the getgamepadbutton constructor uses the get function of hashmap. this hashmap to
        // is used to get a gamepad button for each corresponding gamepad key
        // gamepad keys are enums. they are what you input into the function
        // they are the phisical keys on the controller
        //
        // gamepad button extends button which extends trigger
        //
        // the only difference between gamepad button and button is that gamepad butotn
        // ovverides the get funciton of trigger (and button ig too).
        // here is why it does it:
        // normally when you make a trigger, you need to give it a runnable/command
        // AND a booleansupplier, but
        // if you use a GamepadButotn, you dont have to make a boolean suppplier
        // if you use a button, you can choose to make it a runnable or boolean supplier
        // since Gamepadbutotn overrides the get function of trigger, this boolean supplier
        // is instead handled by the specified gamepad key of the function
        //
        //
        //
        // I think the intention of buttons and gamepad buttons is they are supposed to be like triggers
        // that you can link together
        // first your supposed to make a button or a gamepad button, and either one will return a regular button
        // that you can run anohter function on by typing .
        // you can then run a function such as whenPressed to link another button on
        // these buttons will probably have a runnable/command inside of them
        //

        // i think something like this would be my idea format.
        CommandScheduler.getInstance().addButton(operator.getButton(GamepadKeys.Button.A) ? arm::moveHigh : arm::moveLow);
        // vv
        // addCommand(operatorGet(SLOW_MODE) ? arm::moveHigh : arm::moveLow);
        // or maybe this for something more complicated
        // addCommand(() -> {
        // operatorGet(SLOW_MODE) ? arm::moveHigh : arm::moveLow
        // });

        // I lied now i want this
        // simpleComamnd(operator.get(SLOW_MODE), arm::moveHigh)
        // simpleCOmmand(booleanSupplier condition, Command... command)
        // simpleCOmmand(boolean, interuptable, booleanSupplier condition, Command... command)

        // maybe i could do something like this
//        Command(
//                gamepadbutton,
//                button (aka command)
//                button (aka command)
//        )
//        this could turn into something like this
//        newCommand(
//             operatorGamepad.get(SLOW_MODE),
//             .whenPressed(arm::intake)
//             .whenReleased(arm::outtake)
//        );
//        ^ i think i like this appreach most becuase it is very close. i like that there is a comma to separete, but that isnt required by me
//        i like the shortness ig and its very similar to the default
//        now how would we do this for something like the turert?

//        newCommand(
//             (we could make this condition to run the command be the math to calculatoe the stick
//             touching the edge),
//             .whenPressed(we could od something like
//             turret.setTargetAngle(operator.getx, operator.gety)
//        );

        // maybe i should rename it from when pressed to when active but maybe not







//        operatorGamepad.getGamepadButton(Operator.ARM_INTAKE)
//                .whenPressed(arm::moveHigh)
//                .whenHeld(arm::moveHigh)
//                .whenReleased(arm::moveHigh);
        operator.getGamepadButton(Operator.ARM_RIGHT).whenPressed(arm::moveLow);
        operator.getGamepadButton(Operator.ARM_RIGHT).whenPressed(arm::moveLow);
//        Button hi = () -> {
//            if (operatorGamepad.getTrigger(Operator.ARM_UP) > 0.3 ) {
//                arm.moveHigh();
//            } else {
//                arm.moveLow();
//            }
//        };
        BooleanSupplier qwer = () -> operator.getTrigger(Operator.ARM_UP) > 0.3;
        Button hi = new Button(qwer);
        Button hi = new GamepadTrigger(operator, Operator.ARM_UP).whenPressed(arm::moveHigh);
        CommandScheduler.getInstance().addButton(arm::moveHigh);
//         how to put it on another thread
        CommandScheduler.getInstance().addButton(() -> new Thread(arm::moveHigh).start());

        operator.getGamepadButton(Operator.CAROUSEL).whenPressed(carousel::spinForward);
        CommandScheduler.getInstance().addButton(operator.getButton(GamepadKeys.Button.A) ? arm::moveHigh : arm::moveLow);

        Button armUp = new GamepadTrigger(operator, Operator.ARM_UP).whenPressed(arm::moveHigh);
        waitForStart();
        while (opModeIsActive()) {
            // There are two things that get run when you do this.
            // The periodic method of all defined subsystems, and
            // all of the buttons.
            CommandScheduler.getInstance().run();
        }
    }
}
