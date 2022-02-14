package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//import static org.firstinspires.ftc.teamcode.constants.Constants.ArmConstants.hardware;
import static org.firstinspires.ftc.teamcode.constants.Constants.TurretConstants.*;


public class TurretSubsystem extends HardwareSubsystem {

    private DcMotorEx turret;
    private MotorEx turret2;
    private int targetTicks = 0;
    private double targetDegrees = value.FORWARD;
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
        setDegrees(0);
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
        setDegrees(value.FORWARD);
    }

    public void moveLeft() {
        setDegrees(value.LEFT);
    }

    public void moveRight() {
       setDegrees(value.RIGHT);
    }

    public void moveIn() {
        if (targetDegrees >= 180) {
            setDegrees(value.RETURN + 360);
        }
        setDegrees(value.RETURN + 360);
    }

//    public boolean isZero() {
//        return isZero;
//    }

    public void setTargetXY(double stickX, double stickY) {
//        double position = Math.toDegrees(Math.atan2(stickX, stickY));
//        double min = (360 - controller.RANGE) / 2;
//        double max = 360 - ((360 - controller.RANGE) / 2);
//        setTargetDegrees(Range.clip(position, min, max));
        setDegrees(Math.toDegrees(Math.atan2(stickX, stickY)));
    }
    public void setDegrees(double degrees) {
        targetTicks = (int) ((hardware.CPR / 360) * degrees);
        turret.setTargetPosition(targetTicks);
//        turret2.setTargetPosition((int) ((hardware.CPR / 360) * degrees));
    }
    public void setTargetDegrees(double degrees) {
        targetDegrees = degrees;
    }
    public double getTargetDegrees() {
        return targetDegrees;
    }

    public void moveToTargetDegrees() {
        setDegrees(targetDegrees);
    }

    public boolean isAtTarget() {
        int current   = turret.getCurrentPosition();
//        int tolerance = turret.getTargetPositionTolerance();
        return (current <= value.INITIAL_POSITION + 5 &&
                current >= value.INITIAL_POSITION - 5 &&
                current != 0) || (current <= value.INITIAL_POSITION + 360 + 5 &&
                current >= value.INITIAL_POSITION + 360 - 5 &&
                current != 0);
//        return current == 1;
    }
//
//    public void setTarget(double pos) {
//        turret.setTargetDegrees(pos);
//    }
//    public void resetEncoder() {
//        turret.resetEncoder();
//    }
}