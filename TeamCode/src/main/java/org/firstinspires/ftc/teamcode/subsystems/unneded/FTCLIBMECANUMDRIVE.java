//package org.firstinspires.ftc.teamcode.subsystems.unneded;
//
//import static org.firstinspires.ftc.teamcode.constants.Constants.DriveConstantsBad.*;
//
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.arcrobotics.ftclib.drivebase.MecanumDrive;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.hardware.motors.MotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
//import org.firstinspires.ftc.teamcode.utilities.MotorExEx;
//
//public class FTCLIBMECANUMDRIVE extends SubsystemBase {
//    MecanumDrive mecanumDrive;
//    MecanumDriveSubsystem meca;
//    GamepadEx gamepad;
//    MotorEx leftFront;
//    MotorExEx leftRear;
//    MotorExEx rightFront;
//    MotorExEx rightRear;
//
//    public FTCLIBMECANUMDRIVE(HardwareMap hardwareMap, GamepadEx gamepad) {
//        meca = new MecanumDriveSubsystem(hardwareMap);
//        leftFront = new MotorExEx(hardwareMap, LeftFront.hardware.ID, LeftFront.hardware.CPR, LeftFront.hardware.RPM);
//        leftRear  = new MotorExEx(hardwareMap, LeftRear.hardware.ID, LeftRear.hardware.CPR, LeftRear.hardware.RPM);
//        rightFront= new MotorExEx(hardwareMap, RightFront.hardware.ID, LeftFront.hardware.CPR, LeftFront.hardware.RPM);
//        rightRear = new MotorExEx(hardwareMap, RightRear.hardware.ID, RightRear.hardware.CPR, RightRear.hardware.RPM);
//
//        mecanumDrive = new MecanumDrive(
//                new MotorExEx(hardwareMap, LeftFront.hardware.ID, LeftFront.hardware.CPR, LeftFront.hardware.RPM),
//                new MotorExEx(hardwareMap, LeftRear.hardware.ID, LeftRear.hardware.CPR, LeftRear.hardware.RPM),
//                new MotorExEx(hardwareMap, RightFront.hardware.ID, LeftFront.hardware.CPR, LeftFront.hardware.RPM),
//                new MotorExEx(hardwareMap, RightRear.hardware.ID, RightRear.hardware.CPR, RightRear.hardware.RPM));
//
//        this.gamepad = gamepad;
//
////        leftFront = new MotorExEx(hardwareMap, LeftFront.hardware.ID, LeftFront.hardware.CPR, LeftFront.hardware.RPM);
////        leftRear = new MotorExEx(hardwareMap, LeftRear.hardware.ID, LeftRear.hardware.CPR, LeftRear.hardware.RPM);
////        rightFront = new MotorExEx(hardwareMap, ightFront.hardware.ID, LeftFront.hardware.CPR, LeftFront.hardware.RPM);
////        leftFront = new MotorExEx(hardwareMap, LeftFront.hardware.ID, LeftFront.hardware.CPR, LeftFront.hardware.RPM);
//    }
//
//    @Override
//    public void periodic() {
//        mecanumDrive.driveFieldCentric(gamepad.getLeftX(), gamepad.getLeftY(), gamepad.getRightX(), meca.getRawExternalHeading(), true);
//    }
//}
