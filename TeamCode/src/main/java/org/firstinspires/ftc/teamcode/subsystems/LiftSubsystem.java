package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import static org.firstinspires.ftc.teamcode.constants.Constants.LiftMotorConstants;
import static org.firstinspires.ftc.teamcode.constants.Constants.LiftServoConstants;

public class LiftSubsystem extends HardwareSubsystem{
    DcMotorEx leftLiftMotor;
    DcMotorEx rightLiftMotor;
    ServoEx liftServo;
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

        setAngle(LiftMotorConstants.position.HIGH);
        liftServo.turnToAngle(LiftServoConstants.position.HIGH);

        leftLiftMotor.setPower(LiftMotorConstants.speed.NORMAL_SPEED);
        rightLiftMotor.setPower(LiftMotorConstants.speed.NORMAL_SPEED);
    }

    public void mid() {

        setAngle(LiftMotorConstants.position.MID);
        liftServo.turnToAngle(LiftServoConstants.position.MID);

        leftLiftMotor.setPower(LiftMotorConstants.speed.NORMAL_SPEED);
        rightLiftMotor.setPower(LiftMotorConstants.speed.NORMAL_SPEED);
    }

    public void low() {

        setAngle(LiftMotorConstants.position.LOW);
        liftServo.turnToAngle(LiftServoConstants.position.LOW);

        leftLiftMotor.setPower(LiftMotorConstants.speed.NORMAL_SPEED);
        rightLiftMotor.setPower(LiftMotorConstants.speed.NORMAL_SPEED);
    }

    public void sharedHigh() {

        setAngle(LiftMotorConstants.position.SHARED_HIGH);
        liftServo.turnToAngle(LiftServoConstants.position.SHARED_HIGH);

        leftLiftMotor.setPower(LiftMotorConstants.speed.NORMAL_SPEED);
        rightLiftMotor.setPower(LiftMotorConstants.speed.NORMAL_SPEED);
    }

    public void sharedlow() {

        setAngle(LiftMotorConstants.position.SHARED_LOW);
        liftServo.turnToAngle(LiftServoConstants.position.SHARED_LOW);

        leftLiftMotor.setPower(LiftMotorConstants.speed.NORMAL_SPEED);
        rightLiftMotor.setPower(LiftMotorConstants.speed.NORMAL_SPEED);
    }

    public void initial() {

        setAngle(LiftMotorConstants.position.INITIAL);
        liftServo.turnToAngle(LiftServoConstants.position.INITIAL);

        leftLiftMotor.setPower(LiftMotorConstants.speed.INITIAL_SPEED);
        leftLiftMotor.setPower(LiftMotorConstants.speed.INITIAL_SPEED);
    }

    public void setAngle(double degrees) {
        leftLiftMotor.setTargetPosition((int) (LiftMotorConstants.hardware.CPR / 360 * degrees));
        rightLiftMotor.setTargetPosition((int) (LiftMotorConstants.hardware.CPR / 360 * degrees));
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
