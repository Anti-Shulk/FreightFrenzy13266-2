package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.Constants.ArmConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import static org.firstinspires.ftc.teamcode.constants.DriveConstants.MOTOR_VELO_PID;

public class ArmSubsystem extends HardwareSubsystem {

    private final DcMotorEx arm;
    private int targetTicks;
    private Value.Height height = Value.Height.HIGH;
    private boolean isOut = false;

    public ArmSubsystem() {

        arm = hardwareMap.get(DcMotorEx.class, hardware.ID);
        arm.setPositionPIDFCoefficients(controller.KP);
        arm.setTargetPositionTolerance((int) controller.TOLERANCE);
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

    public void moveShared() {
        setDegrees(value.SHARED);
    }

    public void moveSoItWontHitSides() {
        setDegrees(value.SUS_POSITION);
    }


    public void moveToHeight() {
        switch (height) {
            case LOW: setDegrees(value.LOW); break;
            case MID: setDegrees(value.MID); break;
            case HIGH: setDegrees(value.HIGH); break;
            case AUTO_HIGH: setDegrees(value.AUTO_HIGH); break;
            case AUTO_MID: setDegrees(value.AUTO_MID); break;
            case AUTO_LOW: setDegrees(value.AUTO_LOW); break;
            case SHARED: setDegrees(value.SHARED); break;
        }
    }

    public void moveToNonSusHeight() {
        switch (height) {
            case MID: setDegrees(value.MID); break;
            case HIGH: setDegrees(value.HIGH); break;
            case AUTO_HIGH: setDegrees(value.AUTO_HIGH); break;
            case AUTO_MID: setDegrees(value.AUTO_MID); break;
            default: setDegrees(value.SUS_POSITION); break;
        }
    }

    public void setHeight(Value.Height height) {
        this.height = height;
    }
    public Value.Height getHeight() {
        return height;
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
//        if (getTargetDegrees() < value.SUS_POSITION && !isOut) {
//            setDegrees(value.SUS_POSITION);
//            isOut = true;
//        }
        return getCurrentDegrees() < value.SUS_POSITION - 2;
    }


    public boolean isNotSus() {
        return !isSus();
    }

    public boolean isOut() {
        return isOut;
    }

    public boolean isIn() {
        return !isOut();
    }

    public void setIsOut() {
        this.isOut = true;
    }

    public void setIsIn() {
        this.isOut = false;
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