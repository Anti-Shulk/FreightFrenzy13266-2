package org.firstinspires.ftc.teamcode.subsystems.unneded;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class MecanumDriveSubsystemBasic extends SubsystemBase {
    MecanumDriveSubsystem drive;
    OpMode opMode;
    double denominator;
    double speedMultiplier;


    public MecanumDriveSubsystemBasic(MecanumDriveSubsystem drive) {
        this.drive = drive;
    }

    public void init() {
        drive.setMode(Motor.RunMode.RawPower);
        drive.setMotorPowers(0, 0, 0, 0);
        drive.setPoseEstimate(new Pose2d());
    }

    @Override
    public void periodic() {
        update();
    }

    public void update() {
        drive.update();
    }

    public void mecanumDrive(double forward, double rotate, double strafe, boolean isSlow) {
        denominator = Math.max(Math.abs(forward) + Math.abs(rotate) + Math.abs(strafe), 1);
        speedMultiplier = isSlow ? 0.3 : 1;
        drive.setMotorPowers(
                (((forward + rotate + strafe) / denominator) * speedMultiplier),  // Left Front
                (((forward + rotate - strafe) / denominator) * speedMultiplier),  // Left Rear
                (((forward - rotate - strafe) / denominator) * speedMultiplier),  // Right Front
                (((forward - rotate + strafe) / denominator) * speedMultiplier)); // Right Rear
    }
}
