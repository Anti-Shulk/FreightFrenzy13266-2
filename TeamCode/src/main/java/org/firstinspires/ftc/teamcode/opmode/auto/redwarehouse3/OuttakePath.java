package org.firstinspires.ftc.teamcode.opmode.auto.redwarehouse3;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.opmode.auto.Path;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class OuttakePath extends Path {
    public OuttakePath(MecanumDriveSubsystem drive, Pose2d startPose) {
        super(drive, startPose);
    }

    @Override
    public Trajectory get(double xShift, double yShift) {
        return drive.trajectoryBuilder(new Pose2d(startPose.getX() + xShift, startPose.getY() + yShift), true)
                //                .splineToConstantHeading(new Vector2d(27 + xShift, 73 + yShift), Math.toRadians(-180))
                .back(26 + xShift)
                .splineToConstantHeading(new Vector2d(-12 + xShift, -44 + yShift), Math.toRadians(90)) // second # is end tanject not sure what it does
                .build();
    }
}