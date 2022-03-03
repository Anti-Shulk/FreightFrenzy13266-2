package org.firstinspires.ftc.teamcode.opmode.auto.bluewarehouse3.StartPaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.opmode.auto.Path;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class MidPath extends Path {
    public MidPath(MecanumDriveSubsystem drive, Pose2d startPose) {
        super(drive, startPose);
    }

    @Override
    public Trajectory get(double xShift, double yShift) {
        return null;
    }
}
