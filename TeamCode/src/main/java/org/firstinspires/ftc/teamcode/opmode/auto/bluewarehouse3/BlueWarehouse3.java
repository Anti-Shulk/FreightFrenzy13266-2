package org.firstinspires.ftc.teamcode.opmode.auto.bluewarehouse3;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.TurretArmInQuick;
import org.firstinspires.ftc.teamcode.commands.TurretArmOutQuick;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.opmode.auto.AutoCommands;
import org.firstinspires.ftc.teamcode.opmode.auto.bluewarehouse3.StartPaths.LowPath;
import org.firstinspires.ftc.teamcode.pipeline.DefaultNewDetection;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BoxSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ColorRangeSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HardwareSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TrapdoorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
// TODO: make an auto selectoer
// TODO: make a robot container
public class BlueWarehouse3 extends LinearOpMode {
    Pose2d startPose = new Pose2d(12, 63.2, Math.toRadians(-90));
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
        AutoCommands commands = new AutoCommands(this);



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
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPSIDE_DOWN);

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







        telemetry.update();
        arm.moveSoItWontHitSides();
        waitForStart();




        if (isStopRequested()) return;
//        drive.followTrajectorySequence(lowSeq)
        arm.setHeight(Constants.ArmConstants.Value.Height.AUTO_LOW);
        box.setHeight(Constants.ArmConstants.Value.Height.AUTO_LOW);

        commands.runCommandGroupAsThread(new TurretArmOutQuick(arm, turret, box, turret::moveLeft));

        Trajectory startPath = new LowPath(drive, startPose).get();
        drive.followTrajectory(startPath);

//        sleep(1000);
        trapdoor.open();
//        sleep(1000);

        new Thread(() -> {
            sleep(800);
            commands.runCommandGroup(new TurretArmInQuick(arm, turret, box));
        }).start();


        Trajectory secondPath = new IntakePath(drive, startPath.end(), commands, intake, trapdoor, sensor).get();
        drive.followTrajectory(secondPath);

        double xShift = 0;
        double yShift = 0;
        double intakeDistanceShift = 0;
        arm.setHeight(Constants.ArmConstants.Value.Height.AUTO_HIGH);
        box.setHeight(Constants.ArmConstants.Value.Height.AUTO_HIGH);

        for (int i = 0; i < 5; i++) {
            commands.runCommandGroupAsThread(new TurretArmOutQuick(arm, turret, box, turret::moveLeft, true));

            Trajectory outtakePath = new OuttakePath(drive, secondPath.end()).get(xShift, yShift);
            drive.followTrajectory(outtakePath);
//            sleep(500);
            trapdoor.open();
            trapdoor.open();
//            sleep(500);
            new Thread(() -> {
                trapdoor.open();
                sleep(500);
                commands.runCommandGroup(new TurretArmInQuick(arm, turret, box));
            }).start();


            xShift += 3;
            yShift += 0;
            intakeDistanceShift += 1;

            drive.followTrajectory(new IntakePath(drive, outtakePath.end(), commands, intake, trapdoor, sensor).get(xShift , yShift, intakeDistanceShift));

            xShift = xShift + intakeDistanceShift;
        }
    }

}