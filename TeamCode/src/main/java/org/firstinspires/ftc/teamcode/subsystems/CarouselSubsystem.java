package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.teamcode.constants.Constants.CarouselConstants.Spin;
import static org.firstinspires.ftc.teamcode.constants.Constants.CarouselConstants.Lift;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.MotorExEx;


import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class CarouselSubsystem extends HardwareSubsystem {

//    private CRServo spin;
    private DcMotorEx spin;
//    private ServoEx lift;

//    public CarouselSubsystem(HardwareMap hardwareMap) {
////        spin = new CRServo(hardwareMap, Spin.hardware.ID);
////        spin.setInverted(false);
//
//        lift = new SimpleServo(hardwareMap, Lift.hardware.ID, Lift.hardware.MIN_ANGLE, Lift.hardware.MAX_ANGLE);
//        lift.setInverted(false);
//    }
    public CarouselSubsystem() {
//        spin = new MotorExEx(hardwareMap, Spin.hardware.ID, Spin.hardware.CPR, Spin.hardware.RPM);
        spin = hardwareMap.get(DcMotorEx.class, Spin.hardware.ID);
//        spin2.setPositionCoefficient(controller.KP);
//        spin2.setPositionTolerance(controller.TOLERANCE);
        spin.setDirection(DcMotorSimple.Direction.FORWARD);
//        spin.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        spin2.resetEncoder();
//        spin2.setTargetPosition(Constants.arm.INITIAL_POSITION);
//        spin2.setRunMode(MotorEx.RunMode.PositionControl);
        spin.setVelocity(0);

//        lift = new SimpleServo(hardwareMap, Lift.hardware.ID, Lift.hardware.MIN_ANGLE, Lift.hardware.MAX_ANGLE);
//        lift.setInverted(false);
    }

    @Override
    public void periodic() {

    }

    public void spinFast(boolean reversed) {
        spin.setVelocity(reversed ? -Spin.value.FAST_SPEED : Spin.value.FAST_SPEED);
    }

    public void spinSlow(boolean reversed) {
        spin.setVelocity(reversed ? -Spin.value.SLOW_SPEED : Spin.value.SLOW_SPEED);
    }

    public void spinAuto(boolean reversed) {
        spin.setVelocity(reversed ? -Spin.value.AUTO_SPEED : Spin.value.AUTO_SPEED);
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
    public void spinForwardAuto() {
        spinAuto(false);
    }
    public void spinReversedAuto() {
        spinAuto(true);
    }


    public void lift () {
//        lift.turnToAngle(Lift.value.UP, AngleUnit.DEGREES);
    }
    public void drop () {
//        lift.turnToAngle(Lift.value.DOWN, AngleUnit.DEGREES);
    }

    public void stop() {
        spin.setVelocity(0);
    }
}