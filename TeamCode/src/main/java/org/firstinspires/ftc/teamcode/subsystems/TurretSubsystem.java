package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static org.firstinspires.ftc.teamcode.constants.Constants.TurretConstants.*;


public class TurretSubsystem extends HardwareSubsystem {

    private final DcMotorEx turret;

    private int targetTicks;

    private int adder;

    public TurretSubsystem() {


        turret = hardwareMap.get(DcMotorEx.class, hardware.ID);
//        turret.setPositionPIDFCoefficients(5);
//        turret.setTargetPositionTolerance(10);
        turret.setDirection(hardware.REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDegrees(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(controller.POWER);
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
        if (getTargetDegrees() < -180) {
            setDegrees(-360);
        } else if (getTargetDegrees() > 180 ) {
            setDegrees(360);
        } else {
            setDegrees(0);
        }
    }


    public void setDegrees(double degrees) {
        if (getTargetPosition() == value.INITIAL_POSITION) {
            adder = degrees > 180 ? -360 : 0;
        }

        targetTicks = degreesToTicks(degrees + adder);

        turret.setTargetPosition(targetTicks);
    }

    public int degreesToTicks(double degrees) {
        return (int) ((hardware.CPR / 360) * degrees);
    }

    public double getTargetDegrees() {
        return targetTicks * 360 / hardware.CPR;
    }

    public double getCurrentDegrees() {
        return turret.getCurrentPosition() * 360 / hardware.CPR;
    }



    public void setPosition(int ticks) {
        targetTicks = ticks;
        turret.setTargetPosition(targetTicks);
    }

    public double getTargetPosition() {
        return targetTicks;
    }

    public double getCurrentPosition() {
        return turret.getCurrentPosition();
    }



    public boolean isIn() {
        int current   = turret.getCurrentPosition();
//        int tolerance = turret.getTargetPositionTolerance();
        return (current <= value.INITIAL_POSITION + controller.INTAKE_POSITION_TOLERANCE &&
                current >= value.INITIAL_POSITION - controller.INTAKE_POSITION_TOLERANCE &&
                current != 0) || (current <= value.INITIAL_POSITION + 360 + controller.INTAKE_POSITION_TOLERANCE &&
                current >= value.INITIAL_POSITION + 360 - controller.INTAKE_POSITION_TOLERANCE &&
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
    public void resetEncoder() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetTicks = 0;
    }
}