package org.firstinspires.ftc.teamcode.opmodes.teleop.beta;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@TeleOp(name = "Blue Tele Op Command Op Mode")
public class BlueTeleOpCommandOpMode extends CommandOpMode {

    // Declare Subsystems
    private MecanumDriveSubsystem m_drivetrain;
    private MotorEx fL;



    @Override
    public void initialize() {
        fL = new MotorEx(hardwareMap, "leftFront", Motor.GoBILDA.RPM_435);
        GamepadEx m_operatorGamepad;
        GamepadEx m_driverGamepad;
        m_driverGamepad = new GamepadEx(gamepad1);
        m_operatorGamepad = new GamepadEx(gamepad2);

        // WITH COMMAND BASED
//        OperatorGamepad.getGamepadButton(GamepadKeys.Button.A)
//                .and(OperatorGamepad.getGamepadButton(GamepadKeys.Button.B).negate())
//                .whenActive(new InstantCommand(() -> grip.setPosition(1)));

//        m_operatorGamepad.getGamepadButton(GamepadKeys.Button.A).whenActive(new InstantCommand(() -> drivetrain.drive(1, 1, 1)));
//
//        OperatorGamepad.getGamepadButton(GamepadKeys.Button.B)
//                .and(OperatorGamepad.getGamepadButton(GamepadKeys.Button.A).negate())
//                .whenActive(new InstantCommand(() -> grip.setPosition(0)));

        waitForStart();


        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
        }
    }
}
