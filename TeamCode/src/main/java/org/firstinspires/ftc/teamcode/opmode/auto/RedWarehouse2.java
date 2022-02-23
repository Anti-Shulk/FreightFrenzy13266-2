package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.TurretArmInQuick;
import org.firstinspires.ftc.teamcode.commands.TurretArmOutQuick;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BoxSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
// TODO: make an auto selectoer
// TODO: make a robot container
public class RedWarehouse2 extends LinearOpMode {
    Pose2d startPose = new Pose2d(12, -63.2, Math.toRadians(90));
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
//                .run()
                .build();


//        Pose2d outPos = new Pose2d(-8, 40, Math.toRadians(0));
//        Pose2d inPos = new Pose2d(17, 75, Math.toRadians(0));

//        double xShift = 0;
//        double yShift = 0;

//        Trajectory loop1 = drive.trajectoryBuilder(new Pose2d(17 + xShift, 70 + yShift))
////                .addDisplacementMarker(intake::intake)
//                .forward(30)
//                .build();


//        Trajectory loop1 = drive.trajectoryBuilder(new Pose2d(trajSeq.end().component1() + xShift, trajSeq.end().component2() + yShift), false)
//                .back(30)
////                .addDisplacementMarker(intake::stop)
////                .addDisplacementMarker(runCommandGroupAsThread(new TurretArmOutQuick(arm, turret, box, turret::moveLeft)))
//                .splineToConstantHeading(new Vector2d(-4 + xShift, -40 + yShift), Math.toRadians(180)) // second # is end tanject not sure what it does
//                .build();
//
//        // wait, drop, wait
//
//        Trajectory loop2 = drive.trajectoryBuilder(new Pose2d(loop1.end().component1() + xShift, loop1.end().component2() + yShift))
////                .addDisplacementMarker(runCommandGroupAsThread(new TurretArmInQuick(arm, turret, box)))
//                .splineToConstantHeading(new Vector2d(17 + xShift, -70 + yShift), Math.toRadians(0))
//                .addDisplacementMarker(intake::intake   )
//                .forward(30)
//                .build();


//

//        TrajectorySequence loop = drive.trajectorySequenceBuilder(new Pose2d(17 + xShift, 70 + yShift))
//                .run(intake::intake)
//                .forward(30)
//                .back(40)
//                .addDisplacementMarker(intake::stop)
//                .addDisplacementMarker(runCommandGroupAsThread(new TurretArmOutQuick(arm, turret, box, turret::moveLeft)))
//               .setReversed(false)
//                .lineToLinearHeading(new Pose2d(-8 + xShift, 40 + yShift, Math.toRadians(0)))
//                .waitSeconds(1)
//                //.run(trapdoor::open)
//                .waitSeconds(1)
//                .addDisplacementMarker(runCommandGroupAsThread(new TurretArmInQuick(arm, turret, box)))
//                .lineToLinearHeading(new Pose2d(17 + xShift, 70 + yShift, Math.toRadians(0)))
//                .build();

//                .run(intake::intake)
//                .forward(30)
//                .back(50)
//                .run(intake::stop)
//                .runCommandGroupAsThread(new TurretArmOutQuick(arm, turret, box, turret::moveLeft))
//                .setReversed(false)
//                .lineToLinearHeading(new Pose2d(-6, 45, Math.toRadians(0)))
//                .waitSeconds(1)
////                .run(trapdoor::open)
//                .waitSeconds(1)
//                .runCommandGroupAsThread(new TurretArmInQuick(arm, turret, box))
//                .lineToLinearHeading(new Pose2d(17, 80, Math.toRadians(0)))
//                .run(intake::intake)
//                .forward(30)
//                .back(50)
//                .run(intake::stop)
//                .runCommandGroupAsThread(new TurretArmOutQuick(arm, turret, box, turret::moveLeft))
//                .setReversed(false)
//                .lineToLinearHeading(new Pose2d(-4, 50, Math.toRadians(0)))
//                .waitSeconds(1)
////                .run(trapdoor::open)
//                .waitSeconds(1)
//                .runCommandGroupAsThread(new TurretArmInQuick(arm, turret, box))
//                .lineToLinearHeading(new Pose2d(17, 85, Math.toRadians(0)))
//                .run(intake::intake)
//                .forward(30)
//                .back(50)
//                .run(intake::stop)
//                .runCommandGroupAsThread(new TurretArmOutQuick(arm, turret, box, turret::moveLeft))
//                .setReversed(false)
//                .lineToLinearHeading(new Pose2d(-2, 55, Math.toRadians(0)))
//                .waitSeconds(1)
////                .run(trapdoor::open)
//                .waitSeconds(1)
//                .runCommandGroupAsThread(new TurretArmInQuick(arm, turret, box))
//                .lineToLinearHeading(new Pose2d(17, 90, Math.toRadians(0)))
//                .build();

        waitForStart();

        if (isStopRequested()) return;
        drive.followTrajectorySequence(trajSeq);

        double xShift = 0;
        double yShift = 0;
        while (opModeIsActive()) {
            intake.stop();
            runCommandGroupAsThreadNow(new TurretArmOutQuick(arm, turret, box, turret::moveRight));
//            drive.followTrajectory(loop1);
            drive.followTrajectory(loop1(drive, xShift, yShift));
            runCommandGroupAsThreadNow(new TurretArmInQuick(arm, turret, box));
            drive.followTrajectory(loop2(drive, intake, xShift, yShift));
            xShift += 1.7;
            yShift -= 0;
        }
    }

    public MarkerCallback runCommandGroupAsThread(SequentialCommandGroup sequentialCommandGroup) {

        return () -> new Thread(() -> {
            if (!isStopRequested()) sequentialCommandGroup.initialize();

            while (!isStopRequested() && !sequentialCommandGroup.isFinished()) {
                sequentialCommandGroup.execute();
            }
        }).start();
    }
    public void runCommandGroupAsThreadNow(SequentialCommandGroup sequentialCommandGroup) {
        new Thread(() -> {
            if (!isStopRequested()) sequentialCommandGroup.initialize();

            while (!isStopRequested() && !sequentialCommandGroup.isFinished()) {
                sequentialCommandGroup.execute();
            }
        }).start();
    }

    public TrajectorySequence trajSeq (MecanumDriveSubsystem drive/*, ArmSubsystem arm, TurretSubsystem turret, BoxSubsystem box, IntakeSubsystem intake,*/) {
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-14, -50, Math.toRadians(90)))
//                .runCommandGroupAsThread(new TurretArmInQuick(arm, turret, box))
                .lineToLinearHeading(new Pose2d(17, -70, Math.toRadians(0)))
//                .run(intake::intake)
                .forward(30)
//                .run()
                .build();


        Pose2d loop1End = trajSeq.end();

        return trajSeq;
    }

    public Trajectory loop1 (MecanumDriveSubsystem drive, double xShift, double yShift) {
        Trajectory loop1 = drive.trajectoryBuilder(new Pose2d(trajSeq(drive).end().component1() + xShift, trajSeq(drive).end().component2() + yShift), false)
                .back(30)
//                .addDisplacementMarker(intake::stop)
//                .addDisplacementMarker(runCommandGroupAsThread(new TurretArmOutQuick(arm, turret, box, turret::moveLeft)))
                .splineToConstantHeading(new Vector2d(-6 + xShift, -40 + yShift), Math.toRadians(180)) // second # is end tanject not sure what it does
                .build();

        final Pose2d loop1End = loop1.end();

        return loop1;


    }
    public Trajectory loop2 (MecanumDriveSubsystem drive, IntakeSubsystem intake, double xShift, double yShift) {
        Trajectory loop2 = drive.trajectoryBuilder(new Pose2d(loop1(drive, xShift, yShift).end().component1() + xShift, loop1(drive, xShift, yShift).end().component2() + yShift))
//                .addDisplacementMarker(runCommandGroupAsThread(new TurretArmInQuick(arm, turret, box)))
                .splineToConstantHeading(new Vector2d(17 + xShift, -70 + yShift), Math.toRadians(0))
                .addDisplacementMarker(intake::intake)
                .forward(30)
                .build();

        final Pose2d loop1End = loop2.end();

        return loop2;
    }
}