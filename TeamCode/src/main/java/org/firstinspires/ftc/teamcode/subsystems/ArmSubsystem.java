package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.Constants.ArmConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import static org.firstinspires.ftc.teamcode.constants.DriveConstants.MOTOR_VELO_PID;

public class ArmSubsystem extends HardwareSubsystem {

    private final DcMotorEx arm;
    private int targetTicks;
//    private double targetDegrees = value.HIGH;

    public ArmSubsystem() {

        arm = hardwareMap.get(DcMotorEx.class, hardware.ID);
//        arm.setPositionPIDFCoefficients(5);
//        arm.setTargetPositionTolerance(10);
        arm.setDirection(hardware.REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDegrees(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(controller.TELEOP_POWER);
    }

    @Override
    public void periodic() {
        telemetry.addData("Arm Position", targetTicks + " degrees");
    }

    public void moveHigh() {
        setDegrees(value.HIGH);
    }

    public void moveLow() {
        setDegrees(value.LOW);
    }

    public void moveMid() {
        setDegrees(value.MID);
    }

    public void moveAutoHigh() {
        setDegrees(value.AUTO_HIGH);
    }

    public void moveAutoLow() {
        setDegrees(value.AUTO_LOW);
    }

    public void moveAutoMid() {
        setDegrees(value.AUTO_MID);
    }

    public void moveIn() {
        setDegrees(value.INITIAL);
    }

    public void moveUpSoItWontHitSides() {
        setDegrees(value.MOVE_UP_SO_IT_WONT_HIT_SIDES);
    }

    public void moveShared() {
        setDegrees(value.SHARED);
    }

    
    
    
    public void setDegrees(double degrees) {
        targetTicks = (int) ((hardware.CPR / 360) * degrees);
        arm.setTargetPosition(targetTicks);
    }
    
    public double getTargetDegrees() {
        return targetTicks * 360 / hardware.CPR;
    }
    
    public double getCurrentDegrees() {
        return arm.getCurrentPosition() * 360 / hardware.CPR;
    }
    
    
    
    
    public void setPosition(int ticks) {
        targetTicks = ticks;
        arm.setTargetPosition(targetTicks);
    }

    public double getTargetPosition() {
        return targetTicks;
    }

    public double getCurrentPosition() {
        return arm.getCurrentPosition();
    }

    public boolean isSus() {
        return getCurrentDegrees() < value.MOVE_UP_SO_IT_WONT_HIT_SIDES;
    }


    public void setFullPower() {
        arm.setPower(1);
    }
    public void setOff() {
        arm.setPower(0);
    }


    public void setTeleOpPower() {
        arm.setPower(controller.TELEOP_POWER);
    }
    public void setDropPower() {
        arm.setPower(controller.DROP_POWER);
    }
    public void setAutoPower() {
        arm.setPower(controller.AUTO_POWER);
    }
}