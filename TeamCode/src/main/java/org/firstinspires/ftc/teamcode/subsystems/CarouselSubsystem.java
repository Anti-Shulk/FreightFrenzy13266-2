package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.Constants.Carousel;


public class CarouselSubsystem extends SubsystemBase {

    private CRServo spin;
    private ServoEx lift;

    CarouselSubsystem(HardwareMap hardwareMap) {
        spin = new CRServo(hardwareMap, Carousel.SPIN.NAME);
        spin.setInverted(false);

        lift = new SimpleServo(hardwareMap, Carousel.SPIN.NAME, Carousel.LIFT.MIN_ANGLE, Carousel.LIFT.MAX_ANGLE);
        lift.setInverted(false);
    }

    public void spin (boolean reversed) {
        spin.set(reversed ? -Carousel.SPIN.SPEED : Carousel.SPIN.SPEED);
    }
}
