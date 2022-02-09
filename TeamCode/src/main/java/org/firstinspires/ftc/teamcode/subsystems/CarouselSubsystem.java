package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.teamcode.constants.Constants.Carousel.*;
import static org.firstinspires.ftc.teamcode.constants.Constants.Carousel.Spin.*;
import static org.firstinspires.ftc.teamcode.constants.Constants.Carousel.Lift.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.Constants;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.Constants.Carousel;
import org.firstinspires.ftc.teamcode.utilities.MotorExEx;


public class CarouselSubsystem extends SubsystemBase {

//    private CRServo spin;
    private MotorExEx spin;
    private ServoEx lift;

    public CarouselSubsystem(HardwareMap hardwareMap) {
//        spin = new CRServo(hardwareMap, Spin.hardware.ID);
//        spin.setInverted(false);

        lift = new SimpleServo(hardwareMap, Spin.hardware.ID, Lift.hardware.MIN_ANGLE, Lift.hardware.MAX_ANGLE);
        lift.setInverted(false);
    }
    public CarouselSubsystem(HardwareMap hardwareMap, boolean CRServo) {
        spin = new MotorExEx(hardwareMap, Spin.hardware.ID, Spin.hardware.CPR, Spin.hardware.RPM);
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
        spin.set(reversed ? -Carousel.spin.SPEED : Carousel.spin.SPEED);
    }
    public void spinForward () {
        spin(false);
    }
    public void spinReversed () {
        spin(true);
    }

    public void moveUp () {
        lift.turnToAngle(Carousel.lift.UP, AngleUnit.DEGREES);
    }
    public void moveDown () {
        lift.turnToAngle(Carousel.lift.DOWN, AngleUnit.DEGREES);
    }
}
