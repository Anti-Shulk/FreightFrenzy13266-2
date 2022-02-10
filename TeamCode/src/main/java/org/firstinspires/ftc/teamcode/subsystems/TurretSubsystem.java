package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.constants.Constants.arm;
import static org.firstinspires.ftc.teamcode.constants.Constants.Turret.*;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.utilities.MotorExEx;


public class TurretSubsystem extends SubsystemBase {

    private MotorExEx turret;
    private boolean isZero;
    public TurretSubsystem(HardwareMap hardwareMap) {
        turret = new MotorExEx(hardwareMap, hardware.ID, hardware.CPR, hardware.RPM);
        turret.setPositionCoefficient(controller.KP);
        turret.setPositionTolerance(controller.TOLERANCE);
        turret.setInverted(hardware.REVERSED);
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setRunMode(Motor.RunMode.VelocityControl);
        turret.resetEncoder();
        turret.setTargetPosition(arm.INITIAL_POSITION);
        turret.setRunMode(MotorEx.RunMode.PositionControl);
        turret.set(controller.POWER);
        isZero = false;
    }

    public void moveForward() {
        turret.setTargetDegrees(Constants.turret.FORWARD);
        isZero = true;
    }

    public void moveLeft() {
        turret.setTargetDegrees(Constants.turret.LEFT);
        isZero = true;
    }

    public void moveRight() {
        turret.setTargetDegrees(Constants.turret.RIGHT);
        isZero = true;
    }

    public void moveIntake() {
        turret.setTargetDegrees(Constants.turret.INTAKE);
        isZero = false;
    }

    public boolean isZero() {
        return isZero;
    }

    public void turretMoveIg(double stickX, double stickY) {
        double position = Math.toDegrees(Math.atan2(stickX, stickY));
        double min = (360 - controller.RANGE) / 2;
        double max = 360 - ((360 - controller.RANGE) / 2);
        turret.setTargetDegrees(Range.clip(position, min, max));
    }
    public void setTarget(double pos) {
        turret.setTargetDegrees(pos);
    }
    public void resetEncoder() {
        turret.resetEncoder();
    }
}