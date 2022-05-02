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
public class BlueCarousel4 extends LinearOpMode {
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

                .setReversed(false)
                .splineTo(new Vector2d(-60, 60), Math.toRadians(135))
                .setReversed(true)
                .splineTo(new Vector2d(-45, 23), Math.toRadians(0))
                .back(20)
                .setReversed(false)
                .forward(15)
                .splineTo(new Vector2d(-60, 35), Math.toRadians(90))
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}
