package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.Constants.ArmConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import static org.firstinspires.ftc.teamcode.constants.DriveConstants.MOTOR_VELO_PID;

public class ArmSubsystem extends HardwareSubsystem {

        private final DcMotorEx arm;
//    private final MotorExEx arm;
//    private boolean isIntake;
    private int targetTicks = 0;
    private double targetDegrees = value.HIGH;

    public ArmSubsystem() {
//        arm = new MotorExEx(hardwareMap, hardware.ID, hardware.CPR, hardware.RPM);
//        arm.setPositionCoefficient(controller.KP);
//        arm.setPositionTolerance(controller.TOLERANCE);
//        arm.setInverted(hardware.REVERSED);
//        arm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        arm.setRunMode(Motor.RunMode.VelocityControl);
////        arm.resetEncoder();
//        arm.setTargetPosition(value.INITIAL_POSITION);
//        arm.setRunMode(MotorEx.RunMode.PositionControl);
//        arm.setTargetPosition(value.INITIAL_POSITION);
//        arm.set(controller.POWER);
//        arm.setTargetPosition(value.INITIAL_POSITION);

        arm = hardwareMap.get(DcMotorEx.class, hardware.ID);
//        arm.setPositionPIDFCoefficients(5);
//        arm.setTargetPositionTolerance(10);
        arm.setDirection(hardware.REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDegrees(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(controller.POWER);
    }

    @Override
    public void periodic() {
        telemetry.addData("Arm Position", targetTicks + " degrees");
    }
//    public void setDegrees(double degrees) {
//        arm.setTargetPosition((int) (HARDWARE.CPR * degrees / 360));
////        arm.setPower(ARM.POWER);
//        arm.set(CONTROLLER.POWER);
//    }

    public void moveHigh() {
        setDegrees(value.HIGH);
    }

    public void moveLow() {
        setDegrees(value.LOW);
    }

    public void moveMid() {
        setDegrees(value.MID);
    }

    public void moveAuthHigh() {
        setDegrees(value.AUTO_HIGH);
    }

    public void moveAutoLow() {
        setDegrees(value.AUTO_LOW);
    }

    public void moveAutoMid() {
        setDegrees(value.AUTO_MID);
    }

    public void moveIntake() {
        setDegrees(value.INITIAL);
    }

    public void moveWontHitSides() {
        setDegrees(value.WONT_HIT_SIDES);
    }

    public void moveShared() {
        setDegrees(value.SHARED);
    }

    public void setDegrees(double degrees) {
        targetTicks = (int) ((hardware.CPR / 360) * degrees);
        arm.setTargetPosition(targetTicks);
    }

    public void setTargetDegrees(double targetDegrees) {
        this.targetDegrees = targetDegrees;
        moveToTarget();
    }

    public void getTarget(int target) {
        this.targetDegrees = target;
    }

    public void moveToTarget() {
        setDegrees(targetDegrees);
    }

    public double getTargetTicks() {
        return arm.getCurrentPosition();
    }

    public boolean wontHitSides() {
        int current   = arm.getCurrentPosition();
        return current >= hardware.CPR / 360 * value.WONT_HIT_SIDES;
//        return false;
    }
    public void turnOn() {
        arm.setPower(controller.POWER);
    }
    public void turnLowPower() {
        arm.setPower(controller.LOW_POWER);
    }
    public void turnOff() {
        arm.setPower(0);
    }
    public void turnAutoPower() { arm.setPower(controller.AUTO_POWER);}

//    public boolean isIntake() {
//        return isIntake;
//    }
//    public void setTargetRevolutions(double revolutions) {
//        arm.setTargetPosition((int) (arm.getCPR() * revolutions));
//        double ticksPerRev = 1993;
//        arm2.setTargetPosition((int) (ticksPerRev * revolutions));
//    }
}
