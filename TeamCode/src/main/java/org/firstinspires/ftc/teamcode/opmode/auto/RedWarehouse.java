package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.TurretArmInQuick;
import org.firstinspires.ftc.teamcode.commands.TurretArmOutQuick;
import org.firstinspires.ftc.teamcode.pipeline.DefaultNewDetection;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BoxSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GripperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TrapdoorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
@Disabled
// TODO: make an auto selectoer
// TODO: make a robot container
public class RedWarehouse extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(this);
        HardwareSubsystem hardware = new HardwareSubsystem(this);
        ArmSubsystem arm = new ArmSubsystem();
        TurretSubsystem turret = new TurretSubsystem();
//        TrapdoorSubsystem trapdoor = new TrapdoorSubsystem();
//        GripperSubsystem gripper = new GripperSubsystem();
        IntakeSubsystem intake = new IntakeSubsystem();
        BoxSubsystem box = new BoxSubsystem();

        Pose2d startPose = new Pose2d(3, -63.2, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        /** On initialization **/




//        gripper.moveDown();
//        gripper.close();
//        trapdoor.close();

        arm.setAutoPower();
        arm.moveAutoLow();




        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-14, -50, Math.toRadians(90)))
                .runCommandGroupAsThread(new TurretArmInQuick(arm, turret, box))
                .lineToLinearHeading(new Pose2d(17, -70, Math.toRadians(0)))
                .run(intake::intake)
                .forward(30)
                .back(40)
                .run(intake::stop)
                .runCommandGroupAsThread(new TurretArmOutQuick(arm, turret, box, turret::moveRight))
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-14, -40, Math.toRadians(0)))
                .waitSeconds(1)
//                .run(trapdoor::open)
                .waitSeconds(1)
                .runCommandGroupAsThread(new TurretArmInQuick(arm, turret, box))
                .lineToLinearHeading(new Pose2d(17, -75, Math.toRadians(0)))
                .forward(30)
                .back(50)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-14, -45, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(17, -80, Math.toRadians(0)))
                .forward(30)
                .back(50)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-14, -50, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(17, -85, Math.toRadians(0)))
                .forward(30)
                .back(50)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-14, -55, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(17, -90, Math.toRadians(0)))
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}