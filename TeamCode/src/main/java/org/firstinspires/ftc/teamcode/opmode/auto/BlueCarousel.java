package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.TurretArmOutQuick;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BoxSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.function.BooleanSupplier;

@Autonomous
@Config
public class BlueCarousel extends CommandOpMode {
    public static Vector2d low = new Vector2d(-43, 20);
    public static Vector2d mid = new Vector2d(-40, 20);
    public static Vector2d high = new Vector2d(-35, 20);
    @Override
    public void initialize() {
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(this);
        HardwareSubsystem hardware = new HardwareSubsystem(this);
        ArmSubsystem arm = new ArmSubsystem();
        TurretSubsystem turret = new TurretSubsystem();
        CarouselSubsystem carousel = new CarouselSubsystem();
        BoxSubsystem box = new BoxSubsystem();

        Pose2d startPose = new Pose2d(-40.3, 63.2, Math.toRadians(-90));

        BooleanSupplier ThreadStopper = this::isStopRequested;





        Vector2d selected = high;

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .run(carousel::lift)
                .run(carousel::spinForward)
                .lineToLinearHeading(new Pose2d(-57, 60.5, Math.toRadians(-145)))
                .waitSeconds(3)
                .run(carousel::stop)
                .run(carousel::drop)
                .lineToLinearHeading(new Pose2d(-50, 20, Math.toRadians(-180)))
                .runCommandGroupAsThread(ThreadStopper, new TurretArmOutQuick(arm, turret, box, turret::moveForward, arm::moveHigh))
                .lineToLinearHeading(new Pose2d(selected, Math.toRadians(-180)))
                .setReversed(false)
                .splineTo(new Vector2d(-59, 29), Math.toRadians(90))
                .build();

        waitForStart();
        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}
