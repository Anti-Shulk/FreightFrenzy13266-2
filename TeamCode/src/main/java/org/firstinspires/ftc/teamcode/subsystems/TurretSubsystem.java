package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//import static org.firstinspires.ftc.teamcode.constants.Constants.ArmConstants.hardware;
import static org.firstinspires.ftc.teamcode.constants.Constants.TurretConstants.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.MotorExEx;


public class TurretSubsystem extends HardwareSubsystem {

    private DcMotorEx turret;
    private MotorEx turret2;
    private int target = 0;
//    private boolean isZero;
    public TurretSubsystem() {
//        turret = new MotorExEx(hardwareMap, hardware.ID, hardware.CPR, hardware.RPM);
//        turret.setPositionCoefficient(controller.KP);
//        turret.setPositionTolerance(controller.TOLERANCE);
//        turret.setInverted(hardware.REVERSED);
//        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        turret.setRunMode(Motor.RunMode.VelocityControl);
//        turret.resetEncoder();
//        turret.setTargetDegrees(value.INITIAL_POSITION);
//        turret.setRunMode(MotorEx.RunMode.PositionControl);
//        turret.set(controller.POWER);
//        isZero = false;


        turret = hardwareMap.get(DcMotorEx.class, hardware.ID);
//        turret.setPositionPIDFCoefficients(5);
//        turret.setTargetPositionTolerance(10);
        turret.setDirection(hardware.REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setTargetDegrees(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(controller.POWER);

//        turret2 = new MotorEx(hardwareMap, hardware.ID, hardware.CPR, hardware.RPM);
//        turret2.setInverted(hardware.REVERSED);
//        turret2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        // reset encoder
////        turret2.setTargetPosition(0);
//        turret2.setRunMode(Motor.RunMode.PositionControl);
////        turret2.setTargetPosition(0);
//        turret2.set(controller.POWER);
//        turret2.setPositionTolerance(100);
    }

    @Override
    public void periodic() {

    }

    public void moveForward() {
        setTargetDegrees(value.FORWARD);
    }

    public void moveLeft() {
        setTargetDegrees(value.LEFT);
    }

    public void moveRight() {
       setTargetDegrees(value.RIGHT);
    }

    public void moveDown() {
        setTargetDegrees(value.RETURN);
    }

//    public boolean isZero() {
//        return isZero;
//    }

    public void setTargetXY(double stickX, double stickY) {
//        double position = Math.toDegrees(Math.atan2(stickX, stickY));
//        double min = (360 - controller.RANGE) / 2;
//        double max = 360 - ((360 - controller.RANGE) / 2);
//        setTargetDegrees(Range.clip(position, min, max));
        setTargetDegrees(Math.toDegrees(Math.atan2(stickX, stickY)));
    }
    public void setTargetDegrees(double degrees) {
        target = (int) ((hardware.CPR / 360) * degrees);
        turret.setTargetPosition(target);
//        turret2.setTargetPosition((int) ((hardware.CPR / 360) * degrees));
    }

    public boolean isAtTarget() {
        int current   = turret.getCurrentPosition();
//        int tolerance = turret.getTargetPositionTolerance();
//        return current <= target + 5 &&
//                current >= target - 5;
        return current == 1;
    }
//
//    public void setTarget(double pos) {
//        turret.setTargetDegrees(pos);
//    }
//    public void resetEncoder() {
//        turret.resetEncoder();
//    }
}