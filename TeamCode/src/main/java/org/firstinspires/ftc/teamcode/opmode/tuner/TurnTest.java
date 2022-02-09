package org.firstinspires.ftc.teamcode.opmode.tuner;

import static org.firstinspires.ftc.teamcode.constants.RoadrunnerTuning.turnTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.SampleMecanumDrive;

/*
 * This is a simple routine to test turning capabilities.
 */
//@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
//    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(turnTest.ANGLE));
    }
}
