package org.firstinspires.ftc.teamcode.opmode.teleop.beta;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.GamepadExEx;

@TeleOp(name = "STick test ig")
public class StickTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadExEx tester = new GamepadExEx(gamepad1);


        waitForStart();
        while(opModeIsActive()){
            telemetry.addLine(String.valueOf(
                    Math.toDegrees(Math.atan2(tester.getLeftX(), -tester.getLeftY()))));
            telemetry.addLine(String.valueOf(tester.getRightStickToDegrees()));
            telemetry.addData("left stick x", tester.getLeftX());
            telemetry.addData("left stick y", tester.getLeftY());
            telemetry.addData("right stick x", tester.getRightX());
            telemetry.addData("right stick y", tester.getRightY());
            telemetry.update();
        }
    }
}
