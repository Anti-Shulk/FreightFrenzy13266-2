package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public abstract class Path {
    protected MecanumDriveSubsystem drive;
    protected Pose2d startPose;

    public Path(MecanumDriveSubsystem drive, Pose2d startPose) {
        this.drive = drive;
        this.startPose = startPose;
    }

    public abstract Trajectory get(double xShift, double yShift);

    public Trajectory get() {
        return get(0, 0);
    }
}
