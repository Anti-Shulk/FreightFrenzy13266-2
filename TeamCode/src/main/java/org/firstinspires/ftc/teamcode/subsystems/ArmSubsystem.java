package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.Constants.Arm.*;
import org.firstinspires.ftc.teamcode.constants.Constants;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.utilities.MotorExEx;
//import static org.firstinspires.ftc.teamcode.constants.drive.DriveConstants.MOTOR_VELO_PID;

public class ArmSubsystem extends SubsystemBase {

    //    private final DcMotorEx arm;
    private final MotorExEx arm;
    private boolean isIntake;

    public ArmSubsystem(HardwareMap hardwareMap) {
        arm = new MotorExEx(hardwareMap, hardware.ID, hardware.CPR, hardware.RPM);
        arm.setPositionCoefficient(controller.KP);
        arm.setPositionTolerance(controller.TOLERANCE);
        arm.setInverted(hardware.REVERSED);
        arm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        arm.setRunMode(Motor.RunMode.VelocityControl);
        arm.resetEncoder();
        arm.setTargetPosition(Constants.arm.INITIAL_POSITION);
        arm.setRunMode(MotorEx.RunMode.PositionControl);
        arm.setTargetPosition(Constants.arm.INITIAL_POSITION);
        arm.set(controller.POWER);

//        arm = hMap.get(DcMotorEx.class, HARDWARE.NAME);
//        arm.setPositionPIDFCoefficients(5);
//        arm.setTargetPositionTolerance(10);
//        arm.setDirection(HARDWARE.REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
//        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm.setTargetPosition(0);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm.setPower(ARM.POWER);
    }
//    public void setDegrees(double degrees) {
//        arm.setTargetPosition((int) (HARDWARE.CPR * degrees / 360));
////        arm.setPower(ARM.POWER);
//        arm.set(CONTROLLER.POWER);
//    }

    public void moveHigh() {
        arm.setTargetDegrees(Constants.arm.LOW);
    }

    public void moveLow() {
        arm.setTargetDegrees(Constants.arm.LOW);
    }

    public void moveMid() {
        arm.setTargetDegrees(Constants.arm.MID);
    }

    public void moveIntake() {
        arm.setTargetDegrees(Constants.arm.INTAKE);
    }

    public boolean isIntake() {
        return isIntake;
    }
//    public void setTargetRevolutions(double revolutions) {
//        arm.setTargetPosition((int) (arm.getCPR() * revolutions));
//        double ticksPerRev = 1993;
//        arm2.setTargetPosition((int) (ticksPerRev * revolutions));
//    }
}
