package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;

import org.firstinspires.ftc.teamcode.constants.Constants;

public class IntakeLiftSubsystem extends HardwareSubsystem{
    private final ServoEx lift;
    public IntakeLiftSubsystem() {
        lift = new SimpleServo(hardwareMap, Constants.IntakeLift.hardware.ID, Constants.IntakeLift.hardware.MIN_ANGLE, Constants.IntakeLift.hardware.MAX_ANGLE);
        lift.setInverted(false);
    }

    @Override
    public void periodic() {

    }

    public void lift() {
        lift.turnToAngle(Constants.IntakeLift.value.UP);
    }
    public void drop() {
        lift.turnToAngle(Constants.IntakeLift.value.DOWN);
    }
}
