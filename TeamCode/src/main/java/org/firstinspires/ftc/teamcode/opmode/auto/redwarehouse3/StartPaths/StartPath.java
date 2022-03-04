package org.firstinspires.ftc.teamcode.opmode.auto.redwarehouse3.StartPaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.opmode.auto.Path;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class StartPath extends Path {
    public StartPath(MecanumDriveSubsystem drive, Pose2d startPose) {
        super(drive, startPose);
    }

    @Override
    public Trajectory get(double xShift, double yShift) {
        return get(xShift, yShift, new Pose2d());
    }

    public Trajectory get(double xShift, double yShift, Pose2d targetPose) {
        return drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-10 + xShift, -41 + yShift, Math.toRadians(0)))
                .build();
    }
}

