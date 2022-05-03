package org.firstinspires.ftc.teamcode.opmode.auto.roborodeo;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.IntakeCommandBetter;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.opmode.auto.AutoCommands;
import org.firstinspires.ftc.teamcode.pipeline.RightSideMissingDetection;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ColorRangeSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TrapdoorSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RedCarousel4 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareSubsystem hardware = new HardwareSubsystem(this);
        ColorRangeSensorSubsystem sensor = new ColorRangeSensorSubsystem();
        LiftSubsystem lift = new LiftSubsystem();
        TrapdoorSubsystem trapdoor = new TrapdoorSubsystem();
        CarouselSubsystem carousel = new CarouselSubsystem();
        IntakeSubsystem intake = new IntakeSubsystem();
        AutoCommands autoCommands = new AutoCommands(this);

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
        double forwardDistance = 0;
        MarkerCallback liftCommand = lift::high;




        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(this);

        Pose2d startPose = new Pose2d(-35, -63, Math.toRadians(90));

        drive.setPoseEstimate(startPose);








        while (!isStarted()) {
            telemetry.addData("auto Position", detector.getAnalysis());
            telemetry.addData("value", detector.getNumber());
            telemetry.update();
            switch (detector.getAnalysis()) {
                case LEFT: {
                    forwardDistance = -8.5;
                    liftCommand = lift::low;
                    break;
                }
                case CENTER: {
                    forwardDistance = -6;
                    liftCommand = lift::mid;
                    break;
                }
                default: {
                    forwardDistance = -2;
                    liftCommand = lift::high;
                }
            }
        }

        waitForStart();

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .setConstraints((v, pose2d, pose2d1, pose2d2) -> 20, // Velocity
                        (v, pose2d, pose2d1, pose2d2) -> 30) // Acceleration
                .setTurnConstraint(Math.toRadians(100), //Turn Velocity
                        Math.toRadians(100))  // Turn Acceleration
                .setReversed(false)
                .addDisplacementMarker(carousel::spinReversedAuto)
                //.splineTo(new Vector2d(-63, -50), Math.toRadians(-90))
                .lineToSplineHeading(new Pose2d(-66, -50, Math.toRadians(-90)))
                .forward(2.5)
                .waitSeconds(4)
                .addDisplacementMarker(carousel::stop)
                .setReversed(true)
                .splineTo(new Vector2d(-49, -22), Math.toRadians(0))
                .addDisplacementMarker(liftCommand)
                .back(20 + forwardDistance)
                .waitSeconds(1)
                .addDisplacementMarker(trapdoor::autoDrop)
                .setReversed(false)
                .forward(15 + forwardDistance)
                .addDisplacementMarker(lift::initial)
                .splineTo(new Vector2d(-62, -33), Math.toRadians(-90))
                .build();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}
