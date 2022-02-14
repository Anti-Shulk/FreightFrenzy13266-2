package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetrySubsystem extends SubsystemBase {
    private final Telemetry telemetry;
    private final HardwareSubsystem[] subsystems;
    private MecanumDriveSubsystem drive;
    private final ElapsedTime elapsedTime = new ElapsedTime();
    private int loops = 0;

    public TelemetrySubsystem(Telemetry telemetry, HardwareSubsystem... subsystems) {
        this.telemetry = telemetry;
        this.subsystems = subsystems;
        init();
    }
    public TelemetrySubsystem(Telemetry telemetry, MecanumDriveSubsystem drive, HardwareSubsystem... subsystems){
        this.telemetry = telemetry;
        this.drive = drive;
        this.subsystems = subsystems;
        init();
    }
    
    public void init() {
        if (drive != null) drive.initTelemetry();
        for (HardwareSubsystem subsystem: subsystems) {
            subsystem.init();
        }
        telemetry.update();
    }

    public void teleOpMessage() {
        telemetry.addLine("Good Luck!");
    }

    public void addData(String string, Object value) {
        telemetry.addData(string, String.valueOf(value));
    }

    public void periodic(GamepadEx driver, GamepadEx operator) {
        loops++;
        if (drive != null) drive.initTelemetry();
        for (HardwareSubsystem subsystem: this.subsystems) {
            subsystem.periodic();
        }
        telemetry.addData("Driver Stick",
                  "x = " + driver.getLeftX() +
                        ", y = " + driver.getLeftY() +
                        ", rotate = " + driver.getRightX());
        telemetry.addData("Elapsed time", elapsedTime.toString());
        telemetry.addData("Loops per second", loops / elapsedTime.seconds());
        telemetry.update();
    }
}
