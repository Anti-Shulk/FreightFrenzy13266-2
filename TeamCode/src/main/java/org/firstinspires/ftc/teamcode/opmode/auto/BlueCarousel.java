package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous
public class BlueCarousel extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(this);

        Pose2d startPose = new Pose2d(-40.3, 63.2, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-57, 60.5, Math.toRadians(-135)))
                .splineToLinearHeading(new Pose2d(-40, 23, Math.toRadians(180)), Math.toRadians(0))
                .splineTo(new Vector2d(-59, 35), Math.toRadians(90))
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}
