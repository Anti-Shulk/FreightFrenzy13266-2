package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.Constants.Drive.*;
import static org.firstinspires.ftc.teamcode.constants.Constants.Drive.LeftFront.*;
import static org.firstinspires.ftc.teamcode.constants.Constants.Drive.LeftRear.*;
import static org.firstinspires.ftc.teamcode.constants.Constants.Drive.RightFront.*;
import static org.firstinspires.ftc.teamcode.constants.Constants.Drive.RightRear.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utilities.MotorExEx;

public class FTCLIBMECANUMDRIVE extends SubsystemBase {
    MecanumDrive mecanumDrive;
    SampleMecanumDrive meca;
    GamepadEx gamepad;
    MotorEx leftFront;
    MotorExEx leftRear;
    MotorExEx rightFront;
    MotorExEx rightRear;

    public FTCLIBMECANUMDRIVE(HardwareMap hardwareMap, GamepadEx gamepad) {
        meca = new SampleMecanumDrive(hardwareMap);
        leftFront = new MotorExEx(hardwareMap, LeftFront.hardware.ID, LeftFront.hardware.CPR, LeftFront.hardware.RPM);
        leftRear  = new MotorExEx(hardwareMap, LeftRear.hardware.ID, LeftRear.hardware.CPR, LeftRear.hardware.RPM);
        rightFront= new MotorExEx(hardwareMap, RightFront.hardware.ID, LeftFront.hardware.CPR, LeftFront.hardware.RPM);
        rightRear = new MotorExEx(hardwareMap, RightRear.hardware.ID, RightRear.hardware.CPR, RightRear.hardware.RPM);
        leftFront.

        mecanumDrive = new MecanumDrive(
                new MotorExEx(hardwareMap, LeftFront.hardware.ID, LeftFront.hardware.CPR, LeftFront.hardware.RPM),
                new MotorExEx(hardwareMap, LeftRear.hardware.ID, LeftRear.hardware.CPR, LeftRear.hardware.RPM),
                new MotorExEx(hardwareMap, RightFront.hardware.ID, LeftFront.hardware.CPR, LeftFront.hardware.RPM),
                new MotorExEx(hardwareMap, RightRear.hardware.ID, RightRear.hardware.CPR, RightRear.hardware.RPM));
        this.gamepad = gamepad;

//        leftFront = new MotorExEx(hardwareMap, LeftFront.hardware.ID, LeftFront.hardware.CPR, LeftFront.hardware.RPM);
//        leftRear = new MotorExEx(hardwareMap, LeftRear.hardware.ID, LeftRear.hardware.CPR, LeftRear.hardware.RPM);
//        rightFront = new MotorExEx(hardwareMap, ightFront.hardware.ID, LeftFront.hardware.CPR, LeftFront.hardware.RPM);
//        leftFront = new MotorExEx(hardwareMap, LeftFront.hardware.ID, LeftFront.hardware.CPR, LeftFront.hardware.RPM);
    }

    @Override
    public void periodic() {
        mecanumDrive.driveFieldCentric(gamepad.getLeftX(), gamepad.getLeftY(), gamepad.getRightX(), meca.getRawExternalHeading(), true);
    }
}
