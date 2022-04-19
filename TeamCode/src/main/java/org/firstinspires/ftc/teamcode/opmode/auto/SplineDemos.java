package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Disabled
@Autonomous
public class SplineDemos extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(this);

        Pose2d startPose = new Pose2d(-60, 55, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(0, 20))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-60, 55, Math.toRadians(90)))
                .waitSeconds(1)
                .lineToSplineHeading(new Pose2d(0, 20, Math.toRadians(0)))
                .waitSeconds(1)

                .setReversed(true)
                .splineTo(new Vector2d(-60, 55), Math.toRadians(180))
                .setReversed(false)
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(0, 20), Math.toRadians(0))
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-60, 55, Math.toRadians(-90)), Math.toRadians(180))
                .setReversed(false)
                .waitSeconds(1)
                .splineToSplineHeading(new Pose2d(0, 20, Math.toRadians(90)), Math.toRadians(0))
                .waitSeconds(1)
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}