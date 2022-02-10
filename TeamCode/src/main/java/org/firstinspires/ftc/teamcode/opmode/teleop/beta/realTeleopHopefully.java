package org.firstinspires.ftc.teamcode.opmode.teleop.beta;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.constants.GamepadConstants.*;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CommandSchedulerWrapper;
import org.firstinspires.ftc.teamcode.subsystems.GripperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TrapdoorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.utilities.GamepadExEx;

@TeleOp(name = "Real Teleop Hopefully")
public class realTeleopHopefully extends CommandOpMode {
    @Override
    public void initialize() {
        // TODO: Move to Robot Container
        GamepadExEx driver = new GamepadExEx(gamepad1);
        GamepadExEx operator = new GamepadExEx(gamepad2);

        CommandSchedulerWrapper command = new CommandSchedulerWrapper(this);

        ArmSubsystem arm = new ArmSubsystem(hardwareMap);
        CarouselSubsystem carousel = new CarouselSubsystem(hardwareMap);
        GripperSubsystem gripper = new GripperSubsystem(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(hardwareMap);
        TrapdoorSubsystem trapdoor = new TrapdoorSubsystem(hardwareMap);
        TurretSubsystem turret = new TurretSubsystem(hardwareMap);

        command.addDefault(() -> drive.driveFieldCentric(
                -driver.getLeftY(), -driver.getLeftX(), -driver.getRightX(), false));

        command.add(() -> driver.get(button.SLOW)).whenPressed(drive::setSlow).whenReleased(drive::setNormal);

        command.add(operator::getRightTouchingEdge)
                .whenPressed(() -> turret.setTargetXY(operator.getLeftX(), operator.getLeftY()));

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


        command.add(() -> operator.get(button.ARM_HIGH))
                .whenPressed(arm::moveHigh);
        command.add(() -> operator.get(button.ARM_SHARED))
                .whenPressed(arm::moveShared);

        command.add(() -> operator.get(button.DOWN))
                .whenPressed(turret::moveDown);
        command.add(() -> operator.get(button.FORWARD))
                .whenPressed(turret::moveForward);
        command.add(() -> operator.get(button.LEFT))
                .whenPressed(turret::moveLeft);
        command.add(() -> operator.get(button.RIGHT))
                .whenPressed(turret::moveRight);








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
