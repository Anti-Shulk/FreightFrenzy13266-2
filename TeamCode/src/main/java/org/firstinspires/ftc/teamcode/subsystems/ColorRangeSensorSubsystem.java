package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.Constants;

public class ColorRangeSensorSubsystem extends HardwareSubsystem {
    //TODO: add the code for this
    //TODO: add a command for this maybe 2 idk
    private final RevColorSensorV3 sensor;
    public ColorRangeSensorSubsystem() {
        sensor = hardwareMap.get(RevColorSensorV3.class, Constants.ColorRangeSensorConstants.hardware.ID);
    }
    public boolean detected() {
        return sensor.getDistance(DistanceUnit.MM) < Constants.ColorRangeSensorConstants.value.DISTANCE_THRESHOLD;
    }
    @Override
    public void periodic() {
        telemetry.addData("sensor Distance", sensor.getDistance(DistanceUnit.MM));
    }
}
