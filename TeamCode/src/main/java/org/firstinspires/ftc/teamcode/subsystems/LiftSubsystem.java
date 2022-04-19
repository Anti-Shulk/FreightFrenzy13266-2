package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class LiftSubsystem extends HardwareSubsystem{
    DcMotorEx liftMotor;
    public LiftSubsystem() {
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        liftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(10, 0, 0, 0));
//        liftMotor.setTargetPositionTolerance();
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftMotor.setPower();
    }
//
//    public void high() {
//        liftMotor.setTargetPosition();
//    }
//
//    public void mid() {
//        liftMotor.setTargetPosition();
//    }
//
//    public void low() {
//        liftMotor.setTargetPosition();
//    }
//
//    public void intake() {
//        liftMotor.setTargetPosition();
//    }

    public void setAngle(double degrees) {
        liftMotor.setTargetPosition((int) (384.5 / 360 * degrees));
    }
}
