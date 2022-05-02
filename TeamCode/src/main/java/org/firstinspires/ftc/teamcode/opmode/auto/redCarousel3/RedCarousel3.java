package org.firstinspires.ftc.teamcode.opmode.auto.redCarousel3;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.TurretArmInQuick;
import org.firstinspires.ftc.teamcode.commands.TurretArmOutQuick;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.opmode.auto.AutoCommands;
import org.firstinspires.ftc.teamcode.pipeline.RightSideMissingDetection;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BoxSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ColorRangeSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TrapdoorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous
// TODO: make an auto selectoer
// TODO: make a robot container
public class RedCarousel3 extends LinearOpMode {
    Pose2d startPose = new Pose2d(-40.3, -63.2, Math.toRadians(90));
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(this);
        HardwareSubsystem hardware = new HardwareSubsystem(this);
        ArmSubsystem arm = new ArmSubsystem();
        TurretSubsystem turret = new TurretSubsystem();
        TrapdoorSubsystem trapdoor = new TrapdoorSubsystem();
//        GripperSubsystem gripper = new GripperSubsystem();
        IntakeSubsystem intake = new IntakeSubsystem();
        CarouselSubsystem carousel = new CarouselSubsystem();
        BoxSubsystem box = new BoxSubsystem();
        ColorRangeSensorSubsystem sensor = new ColorRangeSensorSubsystem();
        AutoCommands commands = new AutoCommands(this);


        drive.setPoseEstimate(startPose);


        /** On initialization **/
        trapdoor.close();

        arm.setAutoPower();


        /** Open CV **/


        RightSideMissingDetection detector = new RightSideMissingDetection();


        // Obtain camera id to allow for camera preview
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Obtain webcam name
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Initialize OpenCvWebcam
        // With live preview
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewId);

        // Open the Camera Device Asynchronously
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                // Start Camera Streaming

                // NOTE: this must be called *before* you call startStreaming(...)
                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

                // Start camera stream with 1280x720 resolution
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);

                camera.setPipeline(detector);

            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera status", "Camera failed :(");
            }
        });


        Constants.ArmConstants.Value.Height height = Constants.ArmConstants.Value.Height.AUTO_HIGH;
        Vector2d preLoadVector = new Vector2d(-42, -20);


        while (!isStarted()) {
            telemetry.addData("auto Position", detector.getAnalysis());
            telemetry.addData("value", detector.getNumber());
            telemetry.update();
            switch (detector.getAnalysis()) {
                case LEFT: {
                    height = Constants.ArmConstants.Value.Height.AUTO_LOW;
                    preLoadVector = new Vector2d(-38, -20);
                    break;
                }
                case CENTER: {
                    height = Constants.ArmConstants.Value.Height.AUTO_MID;
                    preLoadVector = new Vector2d(-39, -20);
                    break;
                }
                case RIGHT: {
                    height = Constants.ArmConstants.Value.Height.AUTO_HIGH;
                    preLoadVector  = new Vector2d(-34, -20);
                    break;
                }
            }
        }

        waitForStart();


        TrajectoryVelocityConstraint vel = (v, pose2d, pose2d1, pose2d2) -> 15; // value
        TrajectoryAccelerationConstraint accel = (v, pose2d, pose2d1, pose2d2) -> 15; // value

        TrajectoryVelocityConstraint slowVel = (v, pose2d, pose2d1, pose2d2) -> 15; // value
        TrajectoryAccelerationConstraint slowAccel = (v, pose2d, pose2d1, pose2d2) -> 15; // value

        if (isStopRequested()) return;

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .run(carousel::spinReversedAuto)
                .run(carousel::lift)
//                .setReversed(true)
//                .lineToLinearHeading(new Pose2d(-66, -57, Math.toRadians(-65)), vel, accel)
//                .strafeRight(2)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-66, -50, Math.toRadians(-90)), Math.toRadians(-90), vel, accel)
                .forward(5, slowVel, slowAccel)
                .waitSeconds(3)
                .run(carousel::stop)
                .run(carousel::drop)
                .back(10)
                .build();
//                .lineToLinearHeading(new Pose2d(-50, 20, Math.toRadians(-180)))
        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq.end())
//                .run(() -> commands.runCommandGroupAsThread(new TurretArmOutQuick(arm, turret, box, turret::moveForward)))
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(-45, -20, Math.toRadians(180)), vel, accel)
//                .lineToLinearHeading(preLoadVector)
                .splineToConstantHeading(preLoadVector, Math.toRadians(0), vel, accel)
                .run(trapdoor::open)
                .setReversed(false)
                .splineTo(new Vector2d(-60, -31), Math.toRadians(-90), vel, accel)
                .build();

        waitForStart();
        if (!isStopRequested()) {
            carousel.lift();
            arm.setHeight(height);
            box.setHeight(height);
            drive.followTrajectorySequence(trajSeq);
            commands.runCommandGroup(new TurretArmOutQuick(arm, turret, box, turret::moveForward, true));
            drive.followTrajectorySequence(trajSeq2);
            commands.runCommandGroup(new TurretArmInQuick(arm, turret, box));
        }

    }

}