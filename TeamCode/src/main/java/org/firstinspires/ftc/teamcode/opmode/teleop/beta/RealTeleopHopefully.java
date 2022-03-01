package org.firstinspires.ftc.teamcode.opmode.teleop.beta;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.constants.GamepadConstants.*;

import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.TurretArmInQuick;
import org.firstinspires.ftc.teamcode.commands.TurretArmOutQuick;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BoxSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ColorRangeSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CommandSchedulerWrapper;
import org.firstinspires.ftc.teamcode.subsystems.GripperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TrapdoorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.GamepadExEx;

@TeleOp(name = "TeleOP")
public class RealTeleopHopefully extends CommandOpMode {
    @Override
    public void initialize() {
        // TODO: Move to Robot Container
        GamepadExEx driver = new GamepadExEx(gamepad1);
        GamepadExEx operator = new GamepadExEx(gamepad2);

        CommandSchedulerWrapper command = new CommandSchedulerWrapper();

        HardwareSubsystem hardware = new HardwareSubsystem(this);
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(this);

        ArmSubsystem arm = new ArmSubsystem();
        CarouselSubsystem carousel = new CarouselSubsystem();
        GripperSubsystem gripper = new GripperSubsystem();
        IntakeSubsystem intake = new IntakeSubsystem();
        TrapdoorSubsystem trapdoor = new TrapdoorSubsystem();
        TurretSubsystem turret = new TurretSubsystem();
        ColorRangeSensorSubsystem sensor = new ColorRangeSensorSubsystem();
//        IntakeLiftSubsystem intakeLift = new IntakeLiftSubsystem();
//        ArmTurretSubsystem armTurret = new ArmTurretSubsystem();
        BoxSubsystem box = new BoxSubsystem();


        TelemetrySubsystem telemetrySubsystem = new TelemetrySubsystem(
                telemetry,
                drive,
                carousel,
//                gripper,
                intake,
//                trapdoor,
                arm,
                turret,
                box
        );


//        command.addDefault(() -> telemetry.addData("Driver Stick",
//                  "x = " + driver.getLeftX() +
//                        ", y = " + driver.getLeftY() +
//                        ", rotate = " + driver.getRightX()));
//        command.addDefault(() -> telemetrySubsystem.addData("x", driver.getLeftX()));
//        command.addDefault(() -> telemetrySubsystem.addData("y", driver.getLeftY()));
//        command.addDefault(() -> telemetrySubsystem.addData("rotate", driver.getRightX()));

        command.addDefault(() -> telemetrySubsystem.periodic(driver, operator));

        command.addDefault(() -> drive.drive(
                driver.getLeftX(), driver.getLeftY(), driver.getRightX(), DriveConstants.Drivetrain.Value.FINE_CONTROL, DriveConstants.Drivetrain.Value.FIELD_CENTRIC));

        command.add(() -> driver.get(button.SLOW))
                .whenPressed(drive::setSlow)
                .whenReleased(drive::setNormal);

//        command.add(operator::getRightTouchingEdge)
//                .whileHeld(() -> turret.setTargetXY(operator.getRightX(), -operator.getRightY()));

//        command.add(() -> driver.get(button.TURBO))
//                .whenPressed(drive::setTurbo)
//                .whenReleased(drive::setNormal);

//        command.add(() -> driver.get(button.DROP))
//                .whenPressed(trapdoor::open)
//                .whenReleased(trapdoor::close);

        command.add(() -> driver.get(button.INTAKE))
                .whileHeld(new IntakeCommand(intake, trapdoor, sensor), true)
                .whenReleased(intake::stop);

        command.add(() -> driver.get(button.OUTTAKE))
                .whenPressed(intake::outtake)
                .whenReleased(intake::stop);

        command.add(() -> operator.get(button.DROP))
                .whenPressed(trapdoor::open);

        command.add(() -> operator.get(button.CLOSE))
                .whenPressed(trapdoor::close);

        command.add(()-> driver.get(button.GRIPPER_GRAB))
                .whenPressed(gripper::close)
                .whenReleased(gripper::open);
        command.add(()-> driver.get(button.GRIPPER_LIFT))
                .whenPressed(gripper::moveLow)
                .whenReleased(gripper::moveDown);

//        command.add(() -> operator.get(button.ARM_HIGH))
//                .whenPressed(arm::moveHigh);
//        command.add(() -> operator.get(button.ARM_SHARED))
//                .whenPressed(arm::moveShared);

//        command.add(() -> operator.get(button.DOWN))
//                .whenPressed(turret::moveDown);
//        command.add(() -> operator.get(button.FORWARD))
//                .whenPressed(turret::moveForward);
//        command.add(() -> operator.get(button.LEFT))
//                .whenPressed(turret::moveLeft);
//        command.add(() -> operator.get(button.RIGHT))
//                .whenPressed(turret::moveRight);
















        command.add(() -> operator.get(button.LEFT_SHARED))
                .whenPressed(() -> arm.setHeight(Constants.ArmConstants.Value.Height.SHARED))
                .whenPressed(new TurretArmOutQuick(arm, turret, box, turret::moveLeft), false);
        command.add(() -> operator.get(button.RIGHT_SHARED))
                .whenPressed(() -> arm.setHeight(Constants.ArmConstants.Value.Height.SHARED))
                .whenPressed(new TurretArmOutQuick(arm, turret, box, turret::moveRight), false);
//                .whenReleased(new ArmSetAndMoveToHeight(arm, box, Constants.ArmConstants.Value.Height.HIGH), true);

        command.add(() -> operator.get(button.DOWN))
                .whenPressed(() -> arm.setHeight(Constants.ArmConstants.Value.Height.HIGH))
                .whenPressed(new TurretArmInQuick(arm, turret, box), false);

        command.add(() -> operator.get(button.LEFT))
                .whenPressed(() -> arm.setHeight(Constants.ArmConstants.Value.Height.HIGH))
                .whenPressed(new TurretArmOutQuick(arm, turret, box, turret::moveLeft), false);

        command.add(() -> operator.get(button.RIGHT))
                .whenPressed(() -> arm.setHeight(Constants.ArmConstants.Value.Height.HIGH))
                .whenPressed(new TurretArmOutQuick(arm, turret, box, turret::moveRight), false);

        command.add(() -> operator.get(button.FORWARD))
                .whenPressed(() -> arm.setHeight(Constants.ArmConstants.Value.Height.HIGH))
                .whenPressed(new TurretArmOutQuick(arm, turret, box, turret::moveForward), false);

        command.add(operator::getRightTouchingEdge)
                .whileHeld(() -> telemetry.addLine("pressed"))
                .whileHeld(() -> telemetry.addLine(String.valueOf(operator.getRightStickToDegrees())))
//                .whileHeld(() -> turret.setTargetDegrees())
                .whileHeld(new TurretArmOutQuick(arm, turret, box, () -> turret.setDegrees(operator.getRightStickToDegrees())));














/**
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣴⣶⣿⣿⣷⣶⣄⣀⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⣰⣾⣿⣿⡿⢿⣿⣿⣿⣿⣿⣿⣿⣷⣦⡀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⢀⣾⣿⣿⡟⠁⣰⣿⣿⣿⡿⠿⠻⠿⣿⣿⣿⣿⣧⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⣾⣿⣿⠏⠀⣴⣿⣿⣿⠉⠀⠀⠀⠀⠀⠈⢻⣿⣿⣇⠀⠀⠀
⠀⠀⠀⠀⢀⣠⣼⣿⣿⡏⠀⢠⣿⣿⣿⠇⠀⠀⠀⠀⠀⠀⠀⠈⣿⣿⣿⡀⠀⠀
⠀⠀⠀⣰⣿⣿⣿⣿⣿⡇⠀⢸⣿⣿⣿⡀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⡇⠀⠀
⠀⠀⢰⣿⣿⡿⣿⣿⣿⡇⠀⠘⣿⣿⣿⣧⠀⠀⠀⠀⠀⠀⢀⣸⣿⣿⣿⠁⠀⠀
⠀⠀⣿⣿⣿⠁⣿⣿⣿⡇⠀⠀⠻⣿⣿⣿⣷⣶⣶⣶⣶⣶⣿⣿⣿⣿⠃⠀⠀⠀
⠀⢰⣿⣿⡇⠀⣿⣿⣿⠀⠀⠀⠀⠈⠻⣿⣿⣿⣿⣿⣿⣿⣿⣿⠟⠁⠀⠀⠀⠀
⠀⢸⣿⣿⡇⠀⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠉⠛⠛⠛⠉⢉⣿⣿⠀⠀⠀⠀⠀⠀
⠀⢸⣿⣿⣇⠀⣿⣿⣿⠀⠀⠀⠀⠀⢀⣤⣤⣤⡀⠀⠀⢸⣿⣿⣿⣷⣦⠀⠀⠀
⠀⠀⢻⣿⣿⣶⣿⣿⣿⠀⠀⠀⠀⠀⠈⠻⣿⣿⣿⣦⡀⠀⠉⠉⠻⣿⣿⡇⠀⠀
⠀⠀⠀⠛⠿⣿⣿⣿⣿⣷⣤⡀⠀⠀⠀⠀⠈⠹⣿⣿⣇⣀⠀⣠⣾⣿⣿⡇⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠹⣿⣿⣿⣿⣦⣤⣤⣤⣤⣾⣿⣿⣿⣿⣿⣿⣿⣿⡟⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠻⢿⣿⣿⣿⣿⣿⣿⠿⠋⠉⠛⠋⠉⠉⠁⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠉⠉⠉⠁
**/

//        command.add(() -> operator.get(button.LEFT))
//                .whenPressed(new ArmOutQuick(arm, turret, operator.getRightStickToDegrees()));
//
//        command.add(() -> operator.get(button.FORWARD))
//                .whenPressed(turret::moveForward);



//        command.add(() -> operator.get(button.RIGHT))
//                .whenPressed(turret::moveRight);


        command.add(() -> operator.get(button.CAROUSEL_LIFT))
                .toggleWhenPressed(carousel::lift, carousel::drop);

        command.add(() -> operator.get(button.CAROUSEL_BLUE))
                .whenPressed(carousel::spinForward)
                .whenReleased(carousel::stop);

        command.add(() -> operator.get(button.CAROUSEL_RED))
                .whenPressed(carousel::spinReversed)
                .whenReleased(carousel::stop);

//        command.add(() -> operator.get(GamepadKeys.Button.Y))
//                .whileHeld(gripper::moveHigh);
//        command.add(() -> operator.get(GamepadKeys.Button.BACK))
//                .whileHeld(gripper::open);

//        command.add(() -> driver.get(button.TOGGLE_INTAKE_UP))
//                .toggleWhenPressed(intakeLift::lift, intakeLift::drop);
//        command.add(() -> driver.get(GamepadKeys.Button.A))
//                .whileHeld(gripper::close);







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
