package org.firstinspires.ftc.teamcode.opmode.teleop.beta;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.*;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;
import static org.firstinspires.ftc.teamcode.constants.GamepadConstants.button;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.CarouselCommand;
import org.firstinspires.ftc.teamcode.commands.CarouselStopCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.constants.GamepadConstants;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ColorRangeSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CommandSchedulerWrapper;
import org.firstinspires.ftc.teamcode.subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TrapdoorSubsystem;
import org.firstinspires.ftc.teamcode.util.GamepadExEx;

@TeleOp(name = "Main TeleOP")
public class TeleOPMain extends CommandOpMode {
    @Override
    public void initialize() {
        // TODO: Move to Robot Container
        GamepadExEx driver = new GamepadExEx(gamepad1);
        GamepadExEx operator = new GamepadExEx(gamepad2);

        CommandSchedulerWrapper command = new CommandSchedulerWrapper();

        HardwareSubsystem hardware = new HardwareSubsystem(this);
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(this);
        LiftSubsystem lift = new LiftSubsystem();
        TrapdoorSubsystem trapdoor = new TrapdoorSubsystem();


        TelemetrySubsystem telemetrySubsystem = new TelemetrySubsystem(
                telemetry,
                drive,
                lift);

        command.addDefault(() -> telemetrySubsystem.periodic(driver, operator));

        /*
         *
         * DRIVER COMMANDS
         *
         */

        command.addDefault(() -> drive.drive(
                driver.getLeftX(), driver.getLeftY(), driver.getRightX(), DriveConstants.Drivetrain.Value.FINE_CONTROL, DriveConstants.Drivetrain.Value.FIELD_CENTRIC));

        command.add(() -> driver.get(button.SLOW))
                .whenPressed(drive::setSlow)
                .whenReleased(drive::setNormal);

        command.add(() -> driver.get(button.TURBO))
                .whenPressed(drive::setTurbo)
                .whenReleased(drive::setNormal);

        command.add(() -> driver.get(button.RESET_IMU))
                .whenPressed(drive::resetImu);

        command.add(() -> driver.getTriggerPressed(LEFT_TRIGGER))
                .whenPressed(trapdoor::open);

        command.add(() -> driver.getTriggerPressed(RIGHT_TRIGGER))
                .whenPressed(trapdoor::close);

        /*
         *
         * OPERATOR COMMANDS
         *
         */

        command.add(() -> operator.get(DPAD_DOWN))
                .whenPressed(lift::initial)
                .whenPressed(trapdoor::intake);

        command.add(() -> operator.get(DPAD_UP))
                .whenPressed(lift::high);
        command.add(() -> operator.get(DPAD_LEFT))
                .whenPressed(lift::mid);
        command.add(() -> operator.get(DPAD_RIGHT))
                .whenPressed(lift::low);

        command.add(() -> operator.get(A))
                .whenPressed(trapdoor::open);

        command.add(() -> operator.get(B))
                .whenPressed(trapdoor::close);

        command.add(() -> operator.getLeftY() > GamepadConstants.value.STICK_THRESHOLD)
                .whileHeld(lift::increaseMotorPosition);

        command.add(() -> operator.getLeftY() < -GamepadConstants.value.STICK_THRESHOLD)
                .whileHeld(lift::decreaseMotorPosition);

        command.add(() -> operator.getRightY() > GamepadConstants.value.STICK_THRESHOLD)
                .whileHeld(lift::increaseServoPosition);

        command.add(() -> operator.getRightY() < -GamepadConstants.value.STICK_THRESHOLD)
                .whileHeld(lift::decreaseServoPosition);

        waitForStart();

        while (opModeIsActive()) {
            // There are two things that get run when you do this.
            // The periodic method of all defined subsystems, and
            // the runnable used on the all of the buttons.
            // This runnable will be active based on the get function of
            // the trigger which is why you have to override the get
            // method of Button to be able to use it
            CommandScheduler.getInstance().run();
        }
    }
}
