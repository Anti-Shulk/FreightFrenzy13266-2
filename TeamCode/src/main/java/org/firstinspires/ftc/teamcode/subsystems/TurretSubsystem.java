package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.constants.Constants.TURRET;


public class TurretSubsystem {

    private MotorEx turret;

    public TurretSubsystem(HardwareMap hMap, String name) {
        turret = new MotorEx(hMap, name, TURRET.CPR, TURRET.RPM);
        turret.setInverted(false);
        turret.setTargetPosition(0);
//
//        arm2 = hMap.get(DcMotorEx.class, name);
//        arm2.setPositionPIDFCoefficients(5);
//        arm2.setTargetPositionTolerance(10);
//        arm2.setDirection(DcMotorSimple.Direction.REVERSE);
//        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm2.setTargetPosition(0);
//        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void forward() {
    }
    public void left() {
    }
    public void right() {
    }
    public void down() {
    }
}
