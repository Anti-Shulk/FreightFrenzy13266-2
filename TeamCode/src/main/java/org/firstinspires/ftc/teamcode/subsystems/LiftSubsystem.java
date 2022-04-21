package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import static org.firstinspires.ftc.teamcode.constants.Constants.LiftMotorConstants;
import static org.firstinspires.ftc.teamcode.constants.Constants.LiftServoConstants;

public class LiftSubsystem extends HardwareSubsystem {
    DcMotorEx leftLiftMotor;
    DcMotorEx rightLiftMotor;
    ServoEx liftServo;
    double motorPosition;
    double servoPosition;
    public LiftSubsystem() {
        leftLiftMotor = hardwareMap.get(DcMotorEx.class, LiftMotorConstants.hardware.LEFT_ID);
        leftLiftMotor.setDirection(LiftMotorConstants.hardware.LEFT_REVERSED ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftLiftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(10, 0, 0, 0));
//        leftLiftMotor.setTargetPositionTolerance();



        rightLiftMotor = hardwareMap.get(DcMotorEx.class, LiftMotorConstants.hardware.RIGHT_ID);
        rightLiftMotor.setDirection(LiftMotorConstants.hardware.RIGHT_REVERSED ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftLiftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(10, 0, 0, 0));
//        leftLiftMotor.setTargetPositionTolerance();

        liftServo = new SimpleServo(hardwareMap, LiftServoConstants.hardware.ID, 0, 270);
        liftServo.setInverted(LiftServoConstants.hardware.REVERSED);

        initial();
    }

    public void high() {
        motorPosition = LiftMotorConstants.position.HIGH;
        servoPosition = LiftServoConstants.position.HIGH;
        turnToPositions();
    }

    public void mid() {
        motorPosition = LiftMotorConstants.position.MID;
        servoPosition = LiftServoConstants.position.MID;
        turnToPositions();
    }

    public void low() {
        motorPosition = LiftMotorConstants.position.LOW;
        servoPosition = LiftServoConstants.position.LOW;
        turnToPositions();
    }

    public void sharedHigh() {
        motorPosition = LiftMotorConstants.position.SHARED_HIGH;
        servoPosition = LiftServoConstants.position.SHARED_HIGH;
        turnToPositions();
    }

    public void sharedLow() {
        motorPosition = LiftMotorConstants.position.SHARED_LOW;
        servoPosition = LiftServoConstants.position.SHARED_LOW;
        turnToPositions();
    }

    public void capHigh() {
        motorPosition = LiftMotorConstants.position.CAP_HIGH;
        servoPosition = LiftServoConstants.position.CAP_HIGH;
        turnToPositions();
    }
    public void capLow() {
        motorPosition = LiftMotorConstants.position.CAP_LOW;
        servoPosition = LiftServoConstants.position.CAP_LOW;
        turnToPositions();
    }
    public void capPickUp() {
        motorPosition = LiftMotorConstants.position.CAP_PICKUP;
        servoPosition = LiftServoConstants.position.CAP_PICKUP;
        turnToPositions();
    }

    public void initial() {
        motorPosition = LiftMotorConstants.position.INITIAL;
        servoPosition = LiftServoConstants.position.INITIAL;
        turnToPositions();
    }









    public void setMotorAngle(double degrees) {
        leftLiftMotor.setTargetPosition((int) (LiftMotorConstants.hardware.CPR / 360 * degrees));
        rightLiftMotor.setTargetPosition((int) (LiftMotorConstants.hardware.CPR / 360 * degrees));
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void increaseMotorPosition () {
        changeMotorPosition(LiftMotorConstants.speed.SPEED_DEGREES_CHANGE);
        turnToPositions();
    }

    public void decreaseMotorPosition () {
        changeMotorPosition(-LiftMotorConstants.speed.SPEED_DEGREES_CHANGE);
        turnToPositions();
    }

    public void changeMotorPosition(double degrees) {
        motorPosition += motorPosition >= LiftMotorConstants.position.MAX_POSITION && motorPosition <= LiftMotorConstants.position.MIN_POSITION ? 0 : degrees;
    }



    public void increaseServoPosition () {
        changeServoPosition(LiftServoConstants.speed.SPEED_DEGREES_CHANGE);
        turnToPositions();
    }

    public void decreaseServoPosition () {
        changeServoPosition(-LiftServoConstants.speed.SPEED_DEGREES_CHANGE);
        turnToPositions();
    }

    public void changeServoPosition(double degrees) {
        servoPosition += degrees;
    }

    public void turnToPositions() {
        liftServo.turnToAngle(servoPosition);
        setMotorAngle(motorPosition);

        leftLiftMotor.setPower(LiftMotorConstants.speed.INITIAL_SPEED);
        rightLiftMotor.setPower(LiftMotorConstants.speed.INITIAL_SPEED);
    }

    @Override
    public void periodic() {
        telemetry.addData("Lift Motor position", motorPosition);
        telemetry.addData("Lift Servo position", servoPosition);
    }
}
