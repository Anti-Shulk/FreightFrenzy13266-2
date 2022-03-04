package org.firstinspires.ftc.teamcode.opmode.auto.redCarousel3;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
        Vector2d preLoadVector = new Vector2d(-35, -20);


        while (!isStarted()) {
            telemetry.addData("auto Position", detector.getAnalysis());
            telemetry.update();
            switch (detector.getAnalysis()) {
                case LEFT: {
                    height = Constants.ArmConstants.Value.Height.AUTO_LOW;
                    preLoadVector = new Vector2d(-35, -20);
                    break;
                }
                case CENTER: {
                    height = Constants.ArmConstants.Value.Height.AUTO_MID;
                    preLoadVector = new Vector2d(-35, -20);
                    break;
                }
                case RIGHT: {
                    height = Constants.ArmConstants.Value.Height.AUTO_HIGH;
                    preLoadVector  = new Vector2d(-35, -20);
                    break;
                }
            }
        }

        waitForStart();


        if (isStopRequested()) return;

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .run(carousel::lift)
                .run(carousel::spinForwardSlow)
                .lineToLinearHeading(new Pose2d(-57, -60.5, Math.toRadians(145)))
                .waitSeconds(3)
                .run(carousel::stop)
                .run(carousel::drop)
//                .lineToLinearHeading(new Pose2d(-50, 20, Math.toRadians(-180)))
                .lineToSplineHeading(new Pose2d(-57, -20, Math.toRadians(180)))
                .runCommandGroupAsThread(new TurretArmOutQuick(arm, turret, box, turret::moveForward))
//                .lineToLinearHeading(preLoadVector)
                .splineToConstantHeading(preLoadVector, Math.toRadians(0))
                .setReversed(false)
                .splineTo(new Vector2d(-59, -29), Math.toRadians(-90))
                .build();

        waitForStart();
        if (!isStopRequested()) {
            arm.setHeight(height);
            box.setHeight(height);
            drive.followTrajectorySequence(trajSeq);
        }

    }

}