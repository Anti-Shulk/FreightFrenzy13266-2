package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.teamcode.constants.Constants.CarouselConstants.Spin;
import static org.firstinspires.ftc.teamcode.constants.Constants.CarouselConstants.Lift;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.MotorExEx;


import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;


public class CarouselSubsystem extends HardwareSubsystem {

//    private CRServo spin;
    private MotorExEx spin;
    private ServoEx lift;

//    public CarouselSubsystem(HardwareMap hardwareMap) {
////        spin = new CRServo(hardwareMap, Spin.hardware.ID);
////        spin.setInverted(false);
//
//        lift = new SimpleServo(hardwareMap, Lift.hardware.ID, Lift.hardware.MIN_ANGLE, Lift.hardware.MAX_ANGLE);
//        lift.setInverted(false);
//    }
    public CarouselSubsystem() {
        spin = new MotorExEx(hardwareMap, Spin.hardware.ID, Spin.hardware.CPR, Spin.hardware.RPM);
//        spin2.setPositionCoefficient(controller.KP);
//        spin2.setPositionTolerance(controller.TOLERANCE);
        spin.setInverted(Spin.hardware.REVERSED);
//        spin.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        spin.setRunMode(Motor.RunMode.RawPower);
//        spin2.resetEncoder();
//        spin2.setTargetPosition(Constants.arm.INITIAL_POSITION);
//        spin2.setRunMode(MotorEx.RunMode.PositionControl);
        spin.set(0);

        lift = new SimpleServo(hardwareMap, Lift.hardware.ID, Lift.hardware.MIN_ANGLE, Lift.hardware.MAX_ANGLE);
        lift.setInverted(false);
    }

    @Override
    public void periodic() {

    }

    public void spinFast(boolean reversed) {
        spin.set(reversed ? -Spin.value.FAST_SPEED : Spin.value.FAST_SPEED);
    }

    public void spinSlow(boolean reversed) {
        spin.set(reversed ? -Spin.value.SLOW_SPEED : Spin.value.SLOW_SPEED);
    }

    public void spinAuto(boolean reversed) {
        spin.set(reversed ? -Spin.value.SLOW_SPEED : Spin.value.AUTO_SPEED);
    }

    public void spinForwardFast() {
        spinFast(false);
    }
    public void spinReversedFast() {
        spinFast(true);
    }
    public void spinForwardSlow() {
        spinSlow(false);
    }
    public void spinReversedSlow() {
        spinSlow(true);
    }
    public void SpinForwardAuto() {
        spinAuto(false);
    }
    public void spinReversedAuto() {
        spinAuto(true);
    }


    public void lift () {
        lift.turnToAngle(Lift.value.UP, AngleUnit.DEGREES);
    }
    public void drop () {
        lift.turnToAngle(Lift.value.DOWN, AngleUnit.DEGREES);
    }

    public void stop() {
        spin.set(0);
    }
}