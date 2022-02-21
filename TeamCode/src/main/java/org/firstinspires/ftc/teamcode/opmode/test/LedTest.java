package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LED;

public class LedTest extends LinearOpMode {
    LED led = null;
    ModernRoboticsTouchSensor touchSensor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        led = hardwareMap.get(LED.class, "led");
        touchSensor = hardwareMap.get(ModernRoboticsTouchSensor.class, "touchSensor");
        waitForStart();
        while (opModeIsActive()) {
            led.enable(true);
            led.enableLight(true);
            telemetry.addData("light is on", led.isLightOn());
            telemetry.update();
        }
    }
}
