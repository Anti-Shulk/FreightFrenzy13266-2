package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@TeleOp(name="armtestig")
public class TeleOp2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
//        ArmSubsystem arm = new ArmSubsystem(hardwareMap);
//        TurretSubsystem turret = new TurretSubsystem(hardwareMap);
//        waitForStart();
//        while (opModeIsActive()) {
//            if (gamepad1.dpad_up) {
//                new Thread(() -> armUpCommand(arm, turret, Constants.TurretConstants.value.FORWARD)).start();
//            }
//            if (gamepad1.dpad_left) {
//                new Thread(() -> armUpCommand(arm, turret, Constants.TurretConstants.value.LEFT)).start();
//            }
//            if (gamepad1.dpad_right) {
//                new Thread(() -> armUpCommand(arm, turret, Constants.TurretConstants.value.RIGHT)).start();
//            }
//            if (gamepad1.dpad_down) {
//                new Thread(() -> armDownCommand(arm, turret)).start();
//            }
//
//
//        }
//    }
//    private void armUpCommand(ArmSubsystem arm, TurretSubsystem turret, double pos) {
//        arm.moveHigh();
//        sleep(1000);
//        turret.setTarget(pos);
//    }
//    private void armDownCommand(ArmSubsystem arm, TurretSubsystem turret) {
//        arm.moveHigh();
//        sleep(1000);
//        turret.setTarget(Constants.TurretConstants.value.RETURN);
    }
}
