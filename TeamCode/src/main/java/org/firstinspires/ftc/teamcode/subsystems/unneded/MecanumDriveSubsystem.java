package org.firstinspires.ftc.teamcode.subsystems.unneded;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.util.MotorExEx;

import java.util.List;

/**
 * A subsystem that uses the {@link org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem} class.
 * This periodically calls {@link org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem#update()} which runs the internal
 * state machine for the mecanum drive. All movement/following is async to fit the paradigm.
 */
public class MecanumDriveSubsystem extends SubsystemBase {

    private final org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem drive;
    private final boolean fieldCentric;

    public MecanumDriveSubsystem(org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem drive, boolean isFieldCentric) {
        this.drive = drive;
        fieldCentric = isFieldCentric;
    }

    public void setMode(MotorExEx.RunMode mode) {
        drive.setMode(mode);
    }

    public void setPIDFCoefficients(Motor.RunMode mode, PIDFCoefficients coefficients) {
        drive.setPIDFCoefficients(mode, coefficients);
    }

    public void setPoseEstimate(Pose2d pose) {
        drive.setPoseEstimate(pose);
    }

    public void update() {
        drive.update();
    }

    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }

    public void drive(double leftY, double leftX, double rightX) {
        Pose2d poseEstimate = getPoseEstimate();

        Vector2d input = new Vector2d(-leftY, -leftX).rotated(
                fieldCentric ? -poseEstimate.getHeading() : 0
        );

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -rightX
                )
        );
    }

    public void setDrivePower(Pose2d drivePower) {
        drive.setDrivePower(drivePower);
    }

    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return drive.trajectoryBuilder(startPose);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return drive.trajectoryBuilder(startPose, reversed);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return drive.trajectoryBuilder(startPose, startHeading);
    }

    public void followTrajectory(Trajectory trajectory) {
        drive.followTrajectoryAsync(trajectory);
    }

    public boolean isBusy() {
        return drive.isBusy();
    }

    public void turn(double radians) {
        drive.turnAsync(radians);
    }

    public List<Double> getWheelVelocities() {
        return drive.getWheelVelocities();
    }

    public void stop() {
        drive(0, 0, 0);
    }

    public Pose2d getPoseVelocity() {
        return drive.getPoseVelocity();
    }

    public Localizer getLocalizer() {
        return drive.getLocalizer();
    }

}
