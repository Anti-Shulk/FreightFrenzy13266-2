package org.firstinspires.ftc.teamcode.opmode.auto.roborodeo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class BlueWarehouse extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(this);

        Pose2d startPose = new Pose2d(7.5, 63, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)

                .setReversed(true)
                .splineTo(new Vector2d(-3.15, 41.42), Math.toRadians(-115))









                .setReversed(false)
                .lineToSplineHeading(new Pose2d(6, 57, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(20, 64, Math.toRadians(0)), Math.toRadians(0))
                .forward(30)

                .setReversed(true)
                .back(30)
                .splineTo(new Vector2d(-3.15, 41.42), Math.toRadians(-115))





                .setReversed(false)
                .lineToSplineHeading(new Pose2d(6, 57, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(20, 64, Math.toRadians(0)), Math.toRadians(0))
                .forward(30)

                .setReversed(true)
                .back(30)
                .splineTo(new Vector2d(-3.15, 41.42), Math.toRadians(-115))







                .setReversed(false)
                .lineToSplineHeading(new Pose2d(6, 57, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(20, 64, Math.toRadians(0)), Math.toRadians(0))
                .forward(30)

                .setReversed(true)
                .back(30)
                .splineTo(new Vector2d(-3.15, 41.42), Math.toRadians(-115))







                .setReversed(false)
                .lineToSplineHeading(new Pose2d(6, 57, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(20, 64, Math.toRadians(0)), Math.toRadians(0))
                .forward(30)

                .setReversed(true)
                .back(30)
                .splineTo(new Vector2d(-3.15, 41.42), Math.toRadians(-115))








                .setReversed(false)
                .lineToSplineHeading(new Pose2d(6, 57, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(20, 64, Math.toRadians(0)), Math.toRadians(0))
                .forward(30)

                .setReversed(true)
                .back(30)
                .splineTo(new Vector2d(-3.15, 41.42), Math.toRadians(-115))






                .setReversed(false)
                .lineToSplineHeading(new Pose2d(6, 57, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(20, 64, Math.toRadians(0)), Math.toRadians(0))
                .forward(30)

                .setReversed(true)
                .back(30)
                .splineTo(new Vector2d(-3.15, 41.42), Math.toRadians(-115))


                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}
