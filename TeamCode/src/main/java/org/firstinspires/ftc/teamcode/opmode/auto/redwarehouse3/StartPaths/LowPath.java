package org.firstinspires.ftc.teamcode.opmode.auto.redwarehouse3.StartPaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.opmode.auto.Path;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class LowPath extends Path {
    public LowPath(MecanumDriveSubsystem drive, Pose2d startPose) {
        super(drive, startPose);
    }

    @Override
    public Trajectory get(double xShift, double yShift) {
        return drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-8 + xShift, -42 + yShift, Math.toRadians(0)))
                .build();
    }
}

