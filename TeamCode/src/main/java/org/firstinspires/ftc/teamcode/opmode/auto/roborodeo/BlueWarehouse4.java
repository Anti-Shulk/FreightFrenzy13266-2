package org.firstinspires.ftc.teamcode.opmode.auto.roborodeo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.IntakeCommandBetter;
import org.firstinspires.ftc.teamcode.opmode.auto.AutoCommands;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ColorRangeSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TrapdoorSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class BlueWarehouse4 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareSubsystem hardware = new HardwareSubsystem(this);
        ColorRangeSensorSubsystem sensor = new ColorRangeSensorSubsystem();
        LiftSubsystem lift = new LiftSubsystem();
        TrapdoorSubsystem trapdoor = new TrapdoorSubsystem();
        CarouselSubsystem carousel = new CarouselSubsystem();
        IntakeSubsystem intake = new IntakeSubsystem();
        AutoCommands autoCommands = new AutoCommands(this);



        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(this);

        Pose2d startPose = new Pose2d(7.5, 63, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)

                .setReversed(true)
                .addDisplacementMarker(lift::high)
                .splineTo(new Vector2d(-3.15, 41.42), Math.toRadians(-115))
                .addDisplacementMarker(trapdoor::open)
                .waitSeconds(2)
                .addDisplacementMarker(lift::initial)










                .setReversed(false)
                .lineToSplineHeading(new Pose2d(0, 64, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(20, 70, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(autoCommands.runCommandAsThread(new IntakeCommandBetter(intake, trapdoor, sensor), 4))
                .forward(30)

                .setReversed(true)
                .back(30)
                .splineTo(new Vector2d(-3.15, 41.42), Math.toRadians(-115))





                .setReversed(false)
                .lineToSplineHeading(new Pose2d(0, 64, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(20, 70, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(autoCommands.runCommandAsThread(new IntakeCommandBetter(intake, trapdoor, sensor), 4))
                .forward(30)

                .setReversed(true)
                .back(30)
                .splineTo(new Vector2d(-3.15, 41.42), Math.toRadians(-115))







                .setReversed(false)
                .lineToSplineHeading(new Pose2d(0, 64, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(20, 70, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(autoCommands.runCommandAsThread(new IntakeCommandBetter(intake, trapdoor, sensor), 4))
                .forward(30)

                .setReversed(true)
                .back(30)
                .splineTo(new Vector2d(-3.15, 41.42), Math.toRadians(-115))







                .setReversed(false)
                .lineToSplineHeading(new Pose2d(0, 64, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(20, 70, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(autoCommands.runCommandAsThread(new IntakeCommandBetter(intake, trapdoor, sensor), 4))
                .forward(30)

                .setReversed(true)
                .back(30)
                .splineTo(new Vector2d(-3.15, 41.42), Math.toRadians(-115))








                .setReversed(false)
                .lineToSplineHeading(new Pose2d(0, 64, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(20, 70, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(autoCommands.runCommandAsThread(new IntakeCommandBetter(intake, trapdoor, sensor), 4))
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
