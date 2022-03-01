package org.firstinspires.ftc.teamcode.opmode.auto.redwarehouse3;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.opmode.auto.AutoCommands;
import org.firstinspires.ftc.teamcode.opmode.auto.Path;
import org.firstinspires.ftc.teamcode.subsystems.ColorRangeSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TrapdoorSubsystem;

public class IntakePath extends Path {
    AutoCommands autoCommands;
    IntakeSubsystem intake;
    TrapdoorSubsystem trapdoor;
    ColorRangeSensorSubsystem sensor;
    public IntakePath(MecanumDriveSubsystem drive, Pose2d startPose, AutoCommands autoCommands, IntakeSubsystem intake, TrapdoorSubsystem trapdoor, ColorRangeSensorSubsystem sensor) {
        super(drive, startPose);
        this.autoCommands = autoCommands;
        this.intake = intake;
        this.trapdoor = trapdoor;
        this.sensor = sensor;
    }

    @Override
    public Trajectory get(double xShift, double yShift) {
        Trajectory loop2 = drive.trajectoryBuilder(new Pose2d(startPose.getX(), startPose.getY()), true)
                .addDisplacementMarker(() -> new Thread(() -> {
                    autoCommands.sleep(1000);
                    ElapsedTime elapsedTime = new ElapsedTime();
                    double targetTime = elapsedTime.seconds() + 4;
                    while (targetTime > elapsedTime.seconds()) {
                        autoCommands.runCommandGroup(new IntakeCommand(intake, trapdoor, sensor));
                    }
                    intake.stop();
                }).start())
//                .splineToConstantHeading(new Vector2d(17 + xShift, -80 + yShift), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(-5 + xShift, -55 + yShift))
                .splineToConstantHeading(new Vector2d(17 + xShift, -70 + yShift), Math.toRadians(0))

                .forward(25 + xShift)
                .build();

        final Pose2d loop1End = loop2.end();

        return loop2;
    }
}
