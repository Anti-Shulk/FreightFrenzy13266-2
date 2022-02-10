package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.teamcode.constants.Constants.CarouselConstants.Spin;
import static org.firstinspires.ftc.teamcode.constants.Constants.CarouselConstants.Lift;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.utilities.MotorExEx;


public class CarouselSubsystem extends SubsystemBase {

//    private CRServo spin;
    private CRServo spin;
    private ServoEx lift;

    public CarouselSubsystem(HardwareMap hardwareMap) {
//        spin = new CRServo(hardwareMap, Spin.hardware.ID);
//        spin.setInverted(false);

        lift = new SimpleServo(hardwareMap, Lift.hardware.ID, Lift.hardware.MIN_ANGLE, Lift.hardware.MAX_ANGLE);
        lift.setInverted(false);
    }
    public CarouselSubsystem(HardwareMap hardwareMap, boolean CRServo) {
        spin = new CRServo(hardwareMap, Spin.hardware.ID/*, Spin.hardware.CPR, Spin.hardware.RPM*/);
//        spin2.setPositionCoefficient(controller.KP);
//        spin2.setPositionTolerance(controller.TOLERANCE);
        spin.setInverted(Spin.hardware.REVERSED);
        spin.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        spin.setRunMode(Motor.RunMode.RawPower);
//        spin2.resetEncoder();
//        spin2.setTargetPosition(Constants.arm.INITIAL_POSITION);
//        spin2.setRunMode(MotorEx.RunMode.PositionControl);
        spin.set(0);

        lift = new SimpleServo(hardwareMap, Lift.hardware.ID, Lift.hardware.MIN_ANGLE, Lift.hardware.MAX_ANGLE);
        lift.setInverted(false);
    }

    public void spin (boolean reversed) {
        spin.set(reversed ? -Spin.value.SPEED : Spin.value.SPEED);
    }
    public void spinForward () {
        spin(false);
    }
    public void spinReversed () {
        spin(true);
    }

    public void moveUp () {
        lift.turnToAngle(Lift.value.UP, AngleUnit.DEGREES);
    }
    public void moveDown () {
        lift.turnToAngle(Lift.value.DOWN, AngleUnit.DEGREES);
    }
}