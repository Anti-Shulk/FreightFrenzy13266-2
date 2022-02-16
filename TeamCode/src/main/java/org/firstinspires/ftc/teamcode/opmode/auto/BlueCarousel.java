package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GripperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous
@Config
public class BlueCarousel extends LinearOpMode {
    public static double lowValue = -49;
    public static double midValue = -47;
    public static double highValue = -40;
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(this);
        HardwareSubsystem hardware = new HardwareSubsystem(this);
        ArmSubsystem arm = new ArmSubsystem();
        TurretSubsystem turret = new TurretSubsystem();
        CarouselSubsystem carousel = new CarouselSubsystem();
        GripperSubsystem gripper = new GripperSubsystem();

        Pose2d startPose = new Pose2d(-40.3, 63.2, Math.toRadians(-90));

        Thread outtakeThread = new Thread(() -> {
            while (opModeIsActive()) {
                carousel.drop();
                carousel.stop();
                sleep(500);
                gripper.moveLow();
                sleep(500);
                arm.turnAutoPower();
                arm.moveWontHitSides();
                while (!arm.wontHitSides()) {
                    sleep(50);
                }
                turret.moveToTargetDegrees();
                arm.moveAutoLow();
            }
        });

        Thread armInThread = new Thread(() -> {
            while (opModeIsActive()) {
                sleep(500);
                turret.moveIn();
                gripper.moveDown();
                arm.moveWontHitSides();
                while (!turret.isAtTarget()) {
                    sleep(50);
                }
                arm.turnLowPower();
                arm.moveIntake();
                sleep(500);
                arm.turnOff();
            }
        });




        Vector2d selected = new Vector2d(lowValue, 20);

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(gripper::close)
                .addDisplacementMarker(carousel::lift)
                .addDisplacementMarker(carousel::spinForward)
                .lineToLinearHeading(new Pose2d(-57, 60.5, Math.toRadians(-145)))
                .waitSeconds(3)
                .addDisplacementMarker(outtakeThread::start)
                .lineToLinearHeading(new Pose2d(-45, 20, Math.toRadians(-180)))
                .lineToLinearHeading(new Pose2d(selected, Math.toRadians(-180)))
                .waitSeconds(1)
                .addDisplacementMarker(gripper::open)
                .waitSeconds(1)
                .addDisplacementMarker(armInThread::start)
                .forward(5)
                .waitSeconds(3)
                .setReversed(false)
                .splineTo(new Vector2d(-59, 29), Math.toRadians(90))
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
        sleep(5000);
    }
}