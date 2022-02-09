package org.firstinspires.ftc.teamcode.utilities;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorExEx extends MotorEx {
    /**
     * Constructs an instance motor for the wrapper wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     * @param cpr  the counts per revolution of the motor
     * @param rpm  the revolutions per minute of the motor
     */
    public MotorExEx(@NonNull HardwareMap hMap, String id, double cpr, double rpm) {
        super(hMap, id, cpr, rpm);
        motorEx = (DcMotorEx) super.motor;
    }

    public void setTargetDegrees(double degrees) {
        setTargetPosition((int) (getCPR() * degrees / 360));
    }
}
