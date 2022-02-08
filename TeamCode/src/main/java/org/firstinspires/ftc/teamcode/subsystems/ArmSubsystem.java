package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem extends SubsystemBase {

    private MotorEx arm;
    private DcMotorEx arm2;

    public ArmSubsystem(HardwareMap hMap, String name) {
//        arm = new MotorEx(hMap, name, 1993, 80);
//        arm.setInverted(false);
        arm.setRunMode(MotorEx.RunMode.PositionControl);
//        arm.set

        arm2 = hMap.get(DcMotorEx.class, name);
        arm2.setPositionPIDFCoefficients(5);
        arm2.setTargetPositionTolerance(10);
        arm2.setDirection(DcMotorSimple.Direction.REVERSE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setTargetPosition(0);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void up() {
//        arm.setTargetPosition(1);
//        arm.set(0.2);
        setTargetRevolutions(0.2);
        arm2.setPower(0.3);
    }
    public void down() {
//        setTargetRevolutions(0);
//        arm.set(0.5);
    }
    public void setTargetRevolutions(double revolutions) {
//        arm.setTargetPosition((int) (arm.getCPR() * revolutions));
        double ticksPerRev = 1993;
        arm2.setTargetPosition((int) (ticksPerRev * revolutions));
    }
}
