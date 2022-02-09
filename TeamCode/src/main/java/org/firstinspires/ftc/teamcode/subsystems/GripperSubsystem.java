package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.Constants.Gripper;
import static org.firstinspires.ftc.teamcode.constants.Constants.Gripper.*;
//import static org.firstinspires.ftc.teamcode.constants.Constants.Gripper.Grip.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class GripperSubsystem extends SubsystemBase {
    private ServoEx grip;
    private ServoEx lift;
    public GripperSubsystem(HardwareMap hardwareMap) {
        grip = new SimpleServo(hardwareMap, Grip.hardware.ID, Grip.hardware.MIN_ANGLE, Grip.hardware.MIN_ANGLE);
        grip.setInverted(false);

        lift = new SimpleServo(hardwareMap, Lift.hardware.ID, Lift.hardware.MIN_ANGLE, Lift.hardware.MAX_ANGLE);
        lift.setInverted(false);
    }
    public void open() {
        grip.turnToAngle(Gripper.grip.OPEN , AngleUnit.DEGREES);
    }
    public void close() {
        grip.turnToAngle(Gripper.grip.CLOSE , AngleUnit.DEGREES);
    }

    public void moveUp() {
        lift.turnToAngle(Gripper.lift.UP , AngleUnit.DEGREES);
    }
    public void moveDown() {
        lift.turnToAngle(Gripper.lift.DOWN, AngleUnit.DEGREES);
    }
}
