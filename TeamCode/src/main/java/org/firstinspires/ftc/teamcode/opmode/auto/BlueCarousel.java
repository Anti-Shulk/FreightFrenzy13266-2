package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.ArmOutQuick;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous
@Config
public class BlueCarousel extends LinearOpMode {
    public static Vector2d low = new Vector2d(-43, 20);
    public static Vector2d mid = new Vector2d(-40, 20);
    public static Vector2d high = new Vector2d(-35, 20);
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(this);
        HardwareSubsystem hardware = new HardwareSubsystem(this);
        ArmSubsystem arm = new ArmSubsystem();
        TurretSubsystem turret = new TurretSubsystem();
        CarouselSubsystem carousel = new CarouselSubsystem();

        Pose2d startPose = new Pose2d(-40.3, 63.2, Math.toRadians(-90));





        Vector2d selected = high;

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .run(carousel::lift)
                .run(carousel::spinForward)
                .lineToLinearHeading(new Pose2d(-57, 60.5, Math.toRadians(-145)))
                .waitSeconds(3)
                .lineToLinearHeading(new Pose2d(-50, 20, Math.toRadians(-180)))
                .runCommandGroup(new ArmOutQuick(arm, turret, arm::moveHigh, turret::moveRight))
                .lineToLinearHeading(new Pose2d(selected, Math.toRadians(-180)))
                .setReversed(false)
                .splineTo(new Vector2d(-59, 29), Math.toRadians(90))
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}
