package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.Constants.GrabberConstants.Grip;
import static org.firstinspires.ftc.teamcode.constants.Constants.GrabberConstants.Lift;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class GripperSubsystem extends HardwareSubsystem {
    private final ServoEx grip;
    private final ServoEx lift;
    public GripperSubsystem() {
        grip = new SimpleServo(hardwareMap, Grip.hardware.ID, Grip.hardware.MIN_ANGLE, Grip.hardware.MAX_ANGLE);
        grip.setInverted(false);

        lift = new SimpleServo(hardwareMap, Lift.hardware.ID, Lift.hardware.MIN_ANGLE, Lift.hardware.MAX_ANGLE);
        lift.setInverted(false);
        close();
    }

    @Override
    public void periodic() {

    }

    public void open() {
        grip.turnToAngle(Grip.value.OPEN , AngleUnit.DEGREES);
    }
    public void close() {
        grip.turnToAngle(Grip.value.CLOSE , AngleUnit.DEGREES);
    }

    public void moveHigh() {
        lift.turnToAngle(Lift.value.HIGH, AngleUnit.DEGREES);
    }
    public void moveMid() {
        lift.turnToAngle(Lift.value.MID, AngleUnit.DEGREES);
    }
    public void moveLow() {
        lift.turnToAngle(Lift.value.LOW, AngleUnit.DEGREES);
    }
    public void moveDown() {
        lift.turnToAngle(Lift.value.DOWN, AngleUnit.DEGREES);
    }
}
