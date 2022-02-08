package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.constants.drive.DriveConstants;

@Config
public class Constants {
    public static Arm ARM = new Arm();
    public static Carousel CAROUSEL = new Carousel();
    public static Turret TURRET = new Turret();
    public static Gripper GRIPPER = new Gripper();
    public static Trapdoor TRAPDOOR = new Trapdoor();
    public static CameraServo CAMERASERVO = new CameraServo();


    // TODO: make it work in degrees
    public static class Arm {
        public String NAME                  = "armMotor";
        public boolean REVERSED = false;
        public double RPM = 80;
        public double CPR = 1000;

        public double ARM_HIGH       = 0; // Degrees
        public double ARM_MID        = 0; // Degrees
        public double ARM_LOW        = 0; // Degrees
        public double ARM_INTAKE     = 0; // Degrees
    }

    public static class Turret {
        public String NAME               = "turretMotor";
        public boolean REVERSED = false;
        public double RPM = 80;
        public double CPR = 1000;

        public double TURRET_FORWARD = 0; // Degrees
        public double TURRET_LEFT    = 0; // Degrees
        public double TURRET_RIGHT   = 0; // Degrees
    }

    public static class Intake {
        public String NAME               = "intakeMotor";
        public boolean REVERSED = false;
        public double RPM = 80;
        public double CPR = 1000;

        public double INTAKE_SPEED   = 1; // % of Power
    }

    public static class Carousel {
        public Carousel() { }
        public static Spin SPIN = new Spin();
        public static Lift LIFT = new Lift();

        public static class Spin {
            public String NAME = "carouselServo";
            public boolean REVERSED = false;
            public double RPM = 80;
            public double CPR = 1000;

            public double SPEED                = 30; // Degrees per second
        }
        public static class Lift {
            public String NAME  = "carouselLiftServo";
            public boolean REVERSED = false;
            public double MIN_ANGLE     = 0;
            public double MAX_ANGLE     = 270;

            public double UP                   = 90; // Degrees
            public double DOWN                 = 0; // Degrees
        }




    }

    public static class Gripper {
        public static Grip SPIN = new Grip();
        public static Lift LIFT = new Lift();

        public static class Grip {
            public String GRIPPER_NAME = "gripperServo";
            public double MIN_ANGLE     = 0;
            public double MAX_ANGLE     = 270;
            public boolean REVERSED = false;

            public double OPEN               = 180; // Degrees
            public double CLOSE              = 0; // Degrees
        }
        public static class Lift {
            public String LIFT_NAME = "gripperLiftServo";
            public double MIN_ANGLE     = 0;
            public double MAX_ANGLE     = 270;
            public boolean REVERSED = false;

            public double UP                 = 90; // Degrees
            public double DOWN               = 0; // Degrees
        }


    }

    public static class Trapdoor {
        public String NAME = "trapdoorServo";
        public double MIN_ANGLE     = 0;
        public double MAX_ANGLE     = 270;
        public boolean REVERSED = false;

        public double OPEN                       = 180; // Degrees
        public double CLOSE                      = 0; // Degrees
    }

    public static class CameraServo {

        public String NAME = "cameraServo";
        public double MIN_ANGLE     = 0;
        public double MAX_ANGLE     = 270;
        public boolean REVERSED = false;

        public double BLUE_WAREHOUSE             = 1; // Degrees
        public double BLUE_CAROUSEL              = 1; // Degrees
        public double RED_WAREHOUSE              = 1; // Degrees
        public double RED_CAROUSEL               = 1; // Degrees
    }
}
