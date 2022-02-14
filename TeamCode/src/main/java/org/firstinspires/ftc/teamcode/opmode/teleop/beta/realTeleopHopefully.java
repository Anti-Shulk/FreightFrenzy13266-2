package org.firstinspires.ftc.teamcode.opmode.teleop.beta;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.constants.GamepadConstants.*;

import org.firstinspires.ftc.teamcode.commands.ArmInQuick;
import org.firstinspires.ftc.teamcode.commands.ArmOutQuick;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CommandSchedulerWrapper;
import org.firstinspires.ftc.teamcode.subsystems.GripperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TrapdoorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.GamepadExEx;

@TeleOp(name = "Real Teleop Hopefully")
public class realTeleopHopefully extends CommandOpMode {
    @Override
    public void initialize() {
        // TODO: Move to Robot Container
        GamepadExEx driver = new GamepadExEx(gamepad1);
        GamepadExEx operator = new GamepadExEx(gamepad2);

        CommandSchedulerWrapper command = new CommandSchedulerWrapper(this);

        HardwareSubsystem hardware = new HardwareSubsystem(this);
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(this);

        ArmSubsystem arm = new ArmSubsystem();
        CarouselSubsystem carousel = new CarouselSubsystem();
        GripperSubsystem gripper = new GripperSubsystem();
        IntakeSubsystem intake = new IntakeSubsystem();
        TrapdoorSubsystem trapdoor = new TrapdoorSubsystem();
        TurretSubsystem turret = new TurretSubsystem();
        TelemetrySubsystem telemetrySubsystem = new TelemetrySubsystem(
                telemetry,
                drive,
                arm,
                carousel,
                gripper,
                intake,
                trapdoor,
                turret);

        telemetrySubsystem.teleOpMessage();

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

        command.add(() -> driver.get(button.TURBO))
                .whenPressed(drive::setTurbo)
                .whenReleased(drive::setNormal);

        command.add(() -> driver.get(button.INTAKE))
                .whenPressed(intake::intake)
                .whenReleased(intake::stop);

        command.add(() -> driver.get(button.OUTTAKE))
                .whenPressed(intake::outtake)
                .whenReleased(intake::stop);

        command.add(() -> driver.get(button.DROP))
                .whenPressed(trapdoor::open)
                .whenReleased(trapdoor::close);


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

        command.add(() -> operator.get(button.ARM_SHARED))
                .whenPressed(() -> arm.setTargetDegrees(Constants.ArmConstants.value.SHARED))
                .whenReleased(() -> arm.setTargetDegrees(Constants.ArmConstants.value.HIGH));

        command.add(() -> operator.get(button.DOWN))
                .whenPressed(new ArmInQuick(arm, turret));
//
//        command.add(() -> operator.get(button.LEFT))
//                .whenPressed(new ArmOutQuick(arm, turret, Constants.TurretConstants.value.LEFT));

        command.add(() -> operator.get(button.RIGHT))
                .whenPressed(new ArmOutQuick(arm, turret, Constants.TurretConstants.value.RIGHT));

        command.add(() -> operator.get(button.FORWARD))
                .whenPressed(new ArmOutQuick(arm, turret, Constants.TurretConstants.value.FORWARD));

        command.add(operator::getRightTouchingEdge)
                .whileHeld(() -> telemetry.addLine("pressed"))
                .whileHeld(() -> telemetry.addLine(String.valueOf(operator.getRightStickToDegrees())))
                .whenPressed(new ArmOutQuick(arm, turret, Math.toDegrees(Math.atan2(operator.getRightX(), operator.getRightY()))));

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
                .whenPressed(carousel::stop);







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
