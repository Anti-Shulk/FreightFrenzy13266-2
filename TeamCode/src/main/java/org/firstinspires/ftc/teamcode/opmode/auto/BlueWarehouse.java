package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TrapdoorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Disabled
public class BlueWarehouse extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(this);
        HardwareSubsystem hardware =new HardwareSubsystem(this);
        ArmSubsystem arm=new ArmSubsystem();
        TurretSubsystem turret=new TurretSubsystem();
//        TrapdoorSubsystem trapdoor = new TrapdoorSubsystem();

        Pose2d startPose = new Pose2d(3, 63.2, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        Thread armUp=new Thread(() -> {
            arm.setAutoPower();
            arm.moveAutoHigh();
            while (!arm.isSus()) {
                sleep(50);
            }
            turret.moveLeft();
            });

        Thread armDown=new Thread(() -> {
            turret.moveIn();
            arm.moveSoItWontHitSides();
            while (!turret.isIn()) {
                sleep(50);
            }
            arm.moveIn();
            arm.setDropPower();
            sleep(500);
            arm.setOff();
        });

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .run(armUp::start)
                .lineToLinearHeading(new Pose2d(-14, 38, Math.toRadians(0)))
                .waitSeconds(1)
//                .run(trapdoor::open)
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(17, 69, Math.toRadians(0)))
                .forward(30)
                .back(50)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-14, 38, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(17, 74, Math.toRadians(0)))
                .forward(30)
                .back(50)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-14, 38, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(17, 79, Math.toRadians(0)))
                .forward(30)
                .back(50)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-14, 38, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(17, 84, Math.toRadians(0)))
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}