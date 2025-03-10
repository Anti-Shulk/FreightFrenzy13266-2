package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.TurretArmInQuick;
import org.firstinspires.ftc.teamcode.commands.TurretArmOutQuick;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.pipeline.DefaultNewDetection;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BoxSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ColorRangeSensorSubsystem;
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
public class RedWarehouse2 extends LinearOpMode {
    Pose2d startPose = new Pose2d(12, -63.2, Math.toRadians(90));
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(this);
        HardwareSubsystem hardware = new HardwareSubsystem(this);
        ArmSubsystem arm = new ArmSubsystem();
        TurretSubsystem turret = new TurretSubsystem();
        TrapdoorSubsystem trapdoor = new TrapdoorSubsystem();
//        GripperSubsystem gripper = new GripperSubsystem();
        IntakeSubsystem intake = new IntakeSubsystem();
        BoxSubsystem box = new BoxSubsystem();
        ColorRangeSensorSubsystem sensor = new ColorRangeSensorSubsystem();



        drive.setPoseEstimate(startPose);



        /** Open CV **/


        DefaultNewDetection detector = new DefaultNewDetection();


        // Obtain camera id to allow for camera preview
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Obtain webcam name
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Initialize OpenCvWebcam
        // With live preview
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewId);

        // Open the Camera Device Asynchronously
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                // Start Camera Streaming

                // NOTE: this must be called *before* you call startStreaming(...)
                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

                // Start camera stream with 1280x720 resolution
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);

                camera.setPipeline(detector);

            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera status", "Camera failed :(");
            }
        });

//        sleep(Constants.CameraConstants.value.CAMERA_WAIT_TIME);
        telemetry.addData("auto Position", detector.getAnalysis());



        /** On initialization **/


//        gripper.moveDown();
//        gripper.close();
        trapdoor.close();

        arm.setAutoPower();
        arm.moveAutoLow();


        Trajectory lowSeq1 = drive.trajectoryBuilder(startPose)

//                .run(gripper::moveLow)
                .lineToLinearHeading(new Pose2d(-10, -40, Math.toRadians(0)))
                .build();
//
//        Trajectory lowSeq2 = drive.trajectoryBuilder(lowSeq1.end())
//                .splineToConstantHeading(new Vector2d(17, -70), Math.toRadians(0))
//                .addDisplacementMarker(runCommandGroupAsThread(new IntakeCommand(intake, trapdoor, sensor), 3, intake::stop))
//                .forward(40)
////                .run(trapdoor::close)
////                .run()
//                .build();

        TrajectorySequence midSeq = drive.trajectorySequenceBuilder(startPose)
//                .run(gripper::moveLow)
                .lineToLinearHeading(new Pose2d(-14, -50, Math.toRadians(0)))
//                .run(gripper::open)
                .runCommandGroupAsThread(new TurretArmInQuick(arm, turret, box))
                .lineToLinearHeading(new Pose2d(17, -70, Math.toRadians(0)))
                .runCommandGroupAsThread(new IntakeCommand(intake, trapdoor, sensor), 3, intake::stop)
                .forward(40)
//                .run(trapdoor::close)
//                .run()
                .build();


        TrajectorySequence highSeq = drive.trajectorySequenceBuilder(startPose)
//                .run(gripper::moveLow)
                .lineToLinearHeading(new Pose2d(-14, -50, Math.toRadians(0)))
//                .run(gripper::open)
                .runCommandGroupAsThread(new TurretArmInQuick(arm, turret, box))
                .lineToLinearHeading(new Pose2d(17, -70, Math.toRadians(0)))
                .runCommandGroupAsThread(new IntakeCommand(intake, trapdoor, sensor), 3, intake::stop)
                .forward(40)
//                .run(trapdoor::close)
//                .run()
                .build();



        telemetry.update();
        arm.moveSoItWontHitSides();
        waitForStart();
        double xShift = 0;
        double yShift = 0;



        if (isStopRequested()) return;
//        drive.followTrajectorySequence(lowSeq)
        arm.setHeight(Constants.ArmConstants.Value.Height.AUTO_LOW);
        box.setHeight(Constants.ArmConstants.Value.Height.AUTO_LOW);
        runCommandGroupAsThreadNow(new TurretArmOutQuick(arm, turret, box, turret::moveRight, false));
        drive.followTrajectory(lowSeq1);
        sleep(1000);
        trapdoor.open();
        sleep(1000);
        runCommandGroupAsThreadNow(new TurretArmInQuick(arm, turret, box, 1));
        drive.followTrajectory(loop2(drive, intake, trapdoor, sensor, 0, 0));

        while (opModeIsActive()) {
            arm.setHeight(Constants.ArmConstants.Value.Height.AUTO_HIGH);
            box.setHeight(Constants.ArmConstants.Value.Height.AUTO_HIGH);
            runCommandGroupAsThreadNow(new TurretArmOutQuick(arm, turret, box, turret::moveRight, true));
//            drive.followTrajectory(loop1);
            drive.followTrajectory(loop1(drive, xShift, yShift));
//            sleep(2000);
            trapdoor.open();
//            sleep(2000);
            runCommandGroupAsThreadNow(new TurretArmInQuick(arm, turret, box, 0.7));
            drive.followTrajectory(loop2(drive, intake, trapdoor, sensor, xShift, yShift));
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
    public MarkerCallback runCommandGroupAsThread(SequentialCommandGroup sequentialCommandGroup, double seconds, Runnable stopCommand) {

        return () -> new Thread(() -> {
            ElapsedTime elapsedTime = new ElapsedTime();
            double targetTime = elapsedTime.seconds() + seconds;
            while (targetTime > elapsedTime.seconds()) {
                if (!isStopRequested()) sequentialCommandGroup.initialize();

                while (!isStopRequested() && !sequentialCommandGroup.isFinished()) {
                    sequentialCommandGroup.execute();
                }
            }
            stopCommand.run();
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
    public void runCommandGroupAsThreadNow(SequentialCommandGroup sequentialCommandGroup, double seconds, Runnable stopCommand) {
        new Thread(() -> {
            ElapsedTime elapsedTime = new ElapsedTime();
            double targetTime = elapsedTime.seconds() + seconds;
            while (targetTime > elapsedTime.seconds()) {
                if (!isStopRequested()) sequentialCommandGroup.initialize();

                while (!isStopRequested() && !sequentialCommandGroup.isFinished()) {
                    sequentialCommandGroup.execute();
                }
            }
            stopCommand.run();
        }).start();
    }

    public TrajectorySequence trajSeq (MecanumDriveSubsystem drive/*, ArmSubsystem arm, TurretSubsystem turret, BoxSubsystem box, IntakeSubsystem intake,*/) {
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-14, -50, Math.toRadians(90)))
//                .runCommandGroupAsThread(new TurretArmInQuick(arm, turret, box))
                .splineToLinearHeading(new Pose2d(17, -80, Math.toRadians(0)), Math.toRadians(0))
//                .run(intake::intake)
                .forward(40)
//                .run()
                .build();


        Pose2d loop1End = trajSeq.end();

        return trajSeq;
    }

    public Trajectory loop1 (MecanumDriveSubsystem drive, double xShift, double yShift) {
        Trajectory loop1 = drive.trajectoryBuilder(new Pose2d(trajSeq(drive).end().component1() + xShift, trajSeq(drive).end().component2() + yShift), false)
                .back(40)
//                .addDisplacementMarker(intake::stop)
//                .addDisplacementMarker(runCommandGroupAsThread(new TurretArmOutQuick(arm, turret, box, turret::moveLeft)))
                .splineToConstantHeading(new Vector2d(-6 + xShift, -40 + yShift), Math.toRadians(0)) // second # is end tanject not sure what it does
                .build();

        final Pose2d loop1End = loop1.end();

        return loop1;


    }
    public Trajectory loop2 (MecanumDriveSubsystem drive, IntakeSubsystem intake, TrapdoorSubsystem trapdoor, ColorRangeSensorSubsystem sensor, double xShift, double yShift) {
        Trajectory loop2 = drive.trajectoryBuilder(new Pose2d(loop1(drive, xShift, yShift).end().component1() + xShift, loop1(drive, xShift, yShift).end().component2() + yShift), true)
//                .addDisplacementMarker(runCommandGroupAsThread(new TurretArmInQuick(arm, turret, box)))
                .splineToConstantHeading(new Vector2d(17 + xShift, -80 + yShift), Math.toRadians(0))
                .addDisplacementMarker(() -> runCommandGroupAsThreadNow(new IntakeCommand(intake, trapdoor, sensor), 3, intake::stop))
                .forward(40)
                .build();

        final Pose2d loop1End = loop2.end();

        return loop2;
    }
}