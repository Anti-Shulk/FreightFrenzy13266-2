package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;

import org.firstinspires.ftc.teamcode.constants.Constants;

public class IntakeLiftSubsystem extends HardwareSubsystem{
    private final ServoEx lift;
    public IntakeLiftSubsystem() {
        lift = new SimpleServo(hardwareMap, Constants.IntakeLiftConstants.hardware.ID, Constants.IntakeLiftConstants.hardware.MIN_ANGLE, Constants.IntakeLiftConstants.hardware.MAX_ANGLE);
        lift.setInverted(false);
    }

    @Override
    public void periodic() {

    }

    public void lift() {
        lift.turnToAngle(Constants.IntakeLiftConstants.value.UP);
    }
    public void drop() {
        lift.turnToAngle(Constants.IntakeLiftConstants.value.DOWN);
    }
}
