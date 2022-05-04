package org.firstinspires.ftc.teamcode.opmode.auto.roborodeo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommandBetter;
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
public class BlueWarehouse4 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareSubsystem hardware = new HardwareSubsystem(this);
        ColorRangeSensorSubsystem sensor = new ColorRangeSensorSubsystem();
        LiftSubsystem lift = new LiftSubsystem();
        TrapdoorSubsystem trapdoor = new TrapdoorSubsystem();
        CarouselSubsystem carousel = new CarouselSubsystem();
        IntakeSubsystem intake = new IntakeSubsystem();
        AutoCommands autoCommands = new AutoCommands(this);

        trapdoor.close();

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
        Vector2d startPosition = new Vector2d(-3.15, 41.42); // by default, at high
        MarkerCallback liftCommand = lift::high; // by default, at high



        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(this);

        Pose2d startPose = new Pose2d(10.5, 63, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);


        while (!isStarted()) {
            telemetry.addData("auto Position", detector.getAnalysis());
            telemetry.addData("value", detector.getNumber());
            telemetry.update();
            switch (detector.getAnalysis()) {
                case LEFT: {
                    startPosition = new Vector2d(-3.15, 41.42);
                    liftCommand = lift::low;
                    break;
                }
                case CENTER: {
                    startPosition = new Vector2d(-3.15, 41.42);
                    liftCommand = lift::mid;
                    break;
                }
                default: {
                    startPosition = new Vector2d(-3.15, 41.42);
                    liftCommand = lift::high;
                }
            }
        }

        waitForStart();

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)

                .setReversed(true)
                .addDisplacementMarker(liftCommand)
                .addDisplacementMarker(() -> new Thread(() -> {
                    sleep(1500);
                    trapdoor.autoDrop();
                    sleep(500);
                    lift.initial();
                    trapdoor.intake();
                }).start())
                .lineToSplineHeading(new Pose2d(startPosition, Math.toRadians(65)))
                .waitSeconds(0.3)
                //             .lineToSplineHeading(new Vector2d(-3.15, 41.42), Math.toRadians())
//                .waitSeconds(0.5)








                /** CYCLE 1 */

                .setReversed(false)
                .lineToSplineHeading(new Pose2d(0, 57, Math.toRadians(0)))
                .addDisplacementMarker(() -> autoCommands.runCommandAsThread(new IntakeCommandBetter(intake, trapdoor, sensor), 3))
                .splineToLinearHeading(new Pose2d(20, 65, Math.toRadians(0)), Math.toRadians(0))
                .forward(24)
                .setReversed(true)
                .back(25)
                .addDisplacementMarker(lift::high)
                .addDisplacementMarker(() -> new Thread(() -> {
                    sleep(1600);
                    trapdoor.autoDrop();
                    sleep(500);
                    lift.initial();
                    trapdoor.intake();
                }).start())
                .splineTo(new Vector2d(-3.15, 41.42), Math.toRadians(-115))
                .waitSeconds(0.3)




                /** CYCLE 2 */

                .setReversed(false)
                .lineToSplineHeading(new Pose2d(0, 63, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(20, 71.1, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> autoCommands.runCommandAsThread(new IntakeCommandBetter(intake, trapdoor, sensor), 3))
                .forward(29)
                .setReversed(true)
                .back(27)
                .addDisplacementMarker(lift::high)
                .addDisplacementMarker(() -> new Thread(() -> {
                    sleep(1600);
                    trapdoor.autoDrop();
                    sleep(500);
                    lift.initial();
                    trapdoor.intake();
                }).start())
                .splineTo(new Vector2d(-3.15, 46.42), Math.toRadians(-115))
                .waitSeconds(0.3)




                /** CYCLE 3 */

                .setReversed(false)
                .lineToSplineHeading(new Pose2d(0, 67, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(20, 75, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> autoCommands.runCommandAsThread(new IntakeCommandBetter(intake, trapdoor, sensor), 3))
                .forward(31)
                .setReversed(true)
                .back(29)
                .addDisplacementMarker(lift::high)
                .addDisplacementMarker(() -> new Thread(() -> {
                    sleep(1600);
                    trapdoor.autoDrop();
                    sleep(500);
                    lift.initial();
                    trapdoor.intake();
                }).start())
                .splineTo(new Vector2d(-3.15, 51.42), Math.toRadians(-115))
                .waitSeconds(0.3)





                /** CYCLE 4 */

                .setReversed(false)
                .lineToSplineHeading(new Pose2d(0, 72, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(20, 80, Math.toRadians(0)), Math.toRadians(0))
                //     .addDisplacementMarker(autoCommands.runCommandAsThread(new IntakeCommandBetter(intake, trapdoor, sensor), 4))
                .forward(20)

//                .setReversed(true)
//                .back(30)
//                .splineTo(new Vector2d(-3.15, 41.42), Math.toRadians(-115))
//
//
//
//
//
//
//
//                /** CYCLE 5 */
//
//                .setReversed(false)
//                .lineToSplineHeading(new Pose2d(0, 58, Math.toRadians(0)))
//                .splineToLinearHeading(new Pose2d(20, 66, Math.toRadians(0)), Math.toRadians(0))
//                .addDisplacementMarker(autoCommands.runCommandAsThread(new IntakeCommandBetter(intake, trapdoor, sensor), 4))
//                .forward(30)
//
//                .setReversed(true)
//                .back(30)
//                .splineTo(new Vector2d(-3.15, 41.42), Math.toRadians(-115))


                .build();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}
