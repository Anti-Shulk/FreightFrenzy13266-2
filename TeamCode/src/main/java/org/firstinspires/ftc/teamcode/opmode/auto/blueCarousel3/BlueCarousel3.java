//package org.firstinspires.ftc.teamcode.opmode.auto.blueCarousel3;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.commands.TurretArmInQuick;
//import org.firstinspires.ftc.teamcode.commands.TurretArmOutQuick;
//import org.firstinspires.ftc.teamcode.constants.Constants;
//import org.firstinspires.ftc.teamcode.opmode.auto.AutoCommands;
//import org.firstinspires.ftc.teamcode.pipeline.RightSideMissingDetection;
//import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.BoxSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.CarouselSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.ColorRangeSensorSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.HardwareSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.TrapdoorSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//@Disabled
//@Autonomous
//// TODO: make an auto selectoer
//// TODO: make a robot container
//public class BlueCarousel3 extends LinearOpMode {
//    Pose2d startPose = new Pose2d(-35, 63, Math.toRadians(-90));
//    @Override
//    public void runOpMode() throws InterruptedException {
//        MecanumDriveSubsystem drive = new MecanumDriveSubsystem(this);
//        HardwareSubsystem hardware = new HardwareSubsystem(this);
//        LiftSubsystem liftSubsystem = new LiftSubsystem();
//        TrapdoorSubsystem trapdoor = new TrapdoorSubsystem();
////        GripperSubsystem gripper = new GripperSubsystem();
//        IntakeSubsystem intake = new IntakeSubsystem();
//        CarouselSubsystem carousel = new CarouselSubsystem();
//        BoxSubsystem box = new BoxSubsystem();
//        ColorRangeSensorSubsystem sensor = new ColorRangeSensorSubsystem();
//        AutoCommands commands = new AutoCommands(this);
//
//
//        drive.setPoseEstimate(startPose);
//
//
//        /** On initialization **/
//        trapdoor.close();
//
//
//        /** Open CV **/
//
//
//        RightSideMissingDetection detector = new RightSideMissingDetection();
//
//
//        // Obtain camera id to allow for camera preview
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//        // Obtain webcam name
//        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
//
//        // Initialize OpenCvWebcam
//        // With live preview
//        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewId);
//
//        // Open the Camera Device Asynchronously
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                // Usually this is where you'll want to start streaming from the camera (see section 4)
//                // Start Camera Streaming
//
//                // NOTE: this must be called *before* you call startStreaming(...)
//                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
//
//                // Start camera stream with 1280x720 resolution
//                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
//
//                camera.setPipeline(detector);
//
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                telemetry.addData("Camera status", "Camera failed :(");
//            }
//        });
//
//
//        Constants.ArmConstants.Position.Tall height = Constants.ArmConzzstants.Value.Height.AUTO_HIGH;
//        Vector2d preLoadVector = new Vector2d(-35, 20);
//
//
//        while (!isStarted()) {
//            telemetry.addData("auto Position", detector.getAnalysis());
//            telemetry.addData("value", detector.getNumber());
//            telemetry.update();
//            switch (detector.getAnalysis()) {
//                case LEFT: {
//                    Tall = Constants.LiftMotorConstants.position.TALL;
//                    preLoadVector = new Vector2d(-36, 17);
//                    break;
//                }
//                case CENTER: {
//                    Tall = Constants.LiftMotorConstants.position.MIDDLE;
//                    preLoadVector = new Vector2d(-35, 20);
//                    break;
//                }
//                case RIGHT: {
//                    Tall = Constants.LiftMotorConstants.position.LOWER;
//                    preLoadVector  = new Vector2d(-32, 20);
//                    break;
//                }
//            }
//        }
//
//        waitForStart();
//
//
//        TrajectoryVelocityConstraint vel = (v, pose2d, pose2d1, pose2d2) -> 15; // value
//        TrajectoryAccelerationConstraint accel = (v, pose2d, pose2d1, pose2d2) -> 15; // value
//
//        TrajectoryVelocityConstraint slowVel = (v, pose2d, pose2d1, pose2d2) -> 15; // value
//        TrajectoryAccelerationConstraint slowAccel = (v, pose2d, pose2d1, pose2d2) -> 15; // value
//
//        if (isStopRequested()) return;
//
//        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
//                .run(carousel::spinForwardAuto)
//                .lineToLinearHeading(new Pose2d(-60, 62, Math.toRadians(-145)), vel, accel)
//                .waitSeconds(3)
//                .run(carousel::stop)
//                .lineToSplineHeading(new Pose2d(-59, 40, Math.toRadians(-145)), vel, accel)
//                .build();
////                .lineToLinearHeading(new Pose2d(-50, 20, Math.toRadians(-180)))
//        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq.end())
//                .setReversed(true)
//                .lineToSplineHeading(new Pose2d(-45, 20, Math.toRadians(180)), vel, accel)
////                .lineToLinearHeading(preLoadVector)
//                .splineToConstantHeading(preLoadVector, Math.toRadians(0), vel, accel)
//                .run(trapdoor::open)
//                .setReversed(false)
//                .splineTo(new Vector2d(-65, 30), Math.toRadians(90), vel, accel)
//                .build();
//
//        waitForStart();
//        if (!isStopRequested()) {
//            liftSubsystem.initial();
//            drive.followTrajectorySequence(trajSeq);
//            drive.followTrajectorySequence(trajSeq2);
//
//        }
//
//    }
//
//}