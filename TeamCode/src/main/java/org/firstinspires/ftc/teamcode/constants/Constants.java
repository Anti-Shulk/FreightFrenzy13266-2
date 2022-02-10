package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public static ArmConstants armConstants;
    public static CarouselConstants carouselConstants;
    public static TurretConstants turretConstants;
    public static GrabberConstants grabberConstants;
    public static IntakeConstants intakeConstants;
    public static TrapdoorConstants trapdoorConstants;
    public static CameraServoConstants cameraServoConstants;
    


    // TODO: make it work in degrees
    public static class ArmConstants {
        public static Hardware hardware;
        public static Controller controller;
        public static Value value;

        public static class Hardware {
            public String ID            = "armMotor";
            public boolean REVERSED     = false;
            public double RPM           = 84;
            public double CPR           = 1993;
        }

        public static class Controller {
            public double TOLERANCE     = 10;
            public double KP            = 5;
            public double POWER         = 0.6;
        }
        public static class Value {
            public double HIGH          = 30; // Degrees
            public double MID           = 0; // Degrees
            public double LOW           = 0; // Degrees
            public double INTAKE        = 0; // Degrees

            public int INITIAL_POSITION = 0;
        }
    }

    public static class CameraServoConstants {
        public static Hardware hardware;
        public static Value value;

        public static class Hardware {
            public boolean REVERSED = false;

        }
        public static class Value {
            public double BLUE_WAREHOUSE             = 1; // Degrees
            public double BLUE_CAROUSEL              = 1; // Degrees
            public double RED_WAREHOUSE              = 1; // Degrees
            public double RED_CAROUSEL               = 1; // Degrees
        }
    }

    public static class CarouselConstants {
        public static Spin spin;
        public static Lift lift;

        public static class Spin {
            public static Hardware hardware = new Hardware();
            public static Controller controller = new Controller();
            public static Value value;

            public static class Hardware {
                public String ID = "carouselServo";
                public boolean REVERSED = false;
                public double RPM = 117;
                public double CPR = 1425.1 * 2;
            }
            public static class Controller {
                public double TOLERANCE     = 10;
                public double KP            = 5;
                public double POWER         = 0.6;
            }
            public static class Value {
                public double SPEED                = 30; // Degrees per second
            }


        }
        public static class Lift {
            public static Hardware hardware = new Hardware();
            public static Value value;

            public static class Hardware {
                public String ID = "carouselLiftServo";
                public boolean REVERSED = false;
                public double MIN_ANGLE = 0;
                public double MAX_ANGLE = 270;
            }
            public static class Value {
                public double UP                   = 90; // Degrees
                public double DOWN                 = 0; // Degrees
            }


        }
    }

    public static class DriveConstantsBad {
        public static LeftFront leftFront;
        public static LeftRear leftRear;
        public static RightFront rightFront;
        public static RightRear rightRear;

        public static class LeftFront {
            public static Hardware hardware;

            public static class Hardware {
                public String ID            = "leftFront";
                public boolean REVERSED     = false;
                public double RPM           = 84;
                public double CPR           = 1993;
            }
        }
        public static class LeftRear {
            public static Hardware hardware;

            public static class Hardware {
                public String ID            = "leftRear";
                public boolean REVERSED     = false;
                public double RPM           = 84;
                public double CPR           = 1993;
            }
        }
        public static class RightFront {
            public static Hardware hardware;

            public static class Hardware {
                public String ID            = "rightFront";
                public boolean REVERSED     = false;
                public double RPM           = 84;
                public double CPR           = 1993;
            }
        }
        public static class RightRear {
            public static Hardware hardware;

            public static class Hardware {
                public String ID            = "rightRear";
                public boolean REVERSED     = false;
                public double RPM           = 84;
                public double CPR           = 1993;
            }
        }
    }


    public static class GrabberConstants {
        public static Grip grip;
        public static Lift lift;

        public static class Grip {
            public static Hardware hardware;
            public static Value value;
            public static class Hardware {
                public String ID = "gripperServo";
                public double MIN_ANGLE     = 0;
                public double MAX_ANGLE     = 270;
                public boolean REVERSED = false;
            }
            public static class Value {
                public double OPEN = 180; // Degrees
                public double CLOSE = 0; // Degrees
            }

        }
        public static class Lift {
            public static Hardware hardware;
            public static Value value;
            public static class Hardware {
                public String ID = "gripperLiftServo";
                public double MIN_ANGLE     = 0;
                public double MAX_ANGLE     = 270;
                public boolean REVERSED = false;
            }
            public static class Value {
                public double UP = 90; // Degrees
                public double DOWN = 0; // Degrees
            }

        }
    }
    public static class IntakeConstants {
        public static Hardware hardware;
        public static Controller controller;
        public static Value value;

        public static class Hardware {
            public String ID            = "intakeMotor";
            public boolean REVERSED     = false;
            public double RPM           = 435;
            public double CPR           = 1993; // figure this out
        }

        public static class Controller {
            public double TOLERANCE     = 10;
            public double KP            = 5;
            public double POWER         = 0.6;
            public double INIT_POWER    = 0;
        }
        public static class Value {
            public double INTAKE_SPEED   = 1; // % of Power
        }
    }

    public static class TurretConstants {
        public static Hardware hardware;
        public static Controller controller;
        public static Value value;

        public static class Hardware {
            public String ID               = "turretMotor";
            public boolean REVERSED        = false;
            public double RPM              = 80;
            public double CPR              = 1000;
        }

        public static class Controller {
            public double TOLERANCE     = 10;
            public double KP            = 5;
            public double POWER         = 0.6;
            public double RANGE         = 360; // Degrees

        }
        public static class Value {
            public double FORWARD = 0; // Degrees
            public double LEFT    = 0; // Degrees
            public double RIGHT   = 0; // Degrees
            public double RETURN  = 0; // Degrees

            public int INITIAL_POSITION = 0; // Degrees
        }
    }







    public static class TrapdoorConstants {
        public static Hardware hardware;
        public static Value value;

        public static class Hardware {
            public String ID = "trapdoorServo";
            public double MIN_ANGLE = 0;
            public double MAX_ANGLE = 270;
            public boolean REVERSED = false;
        }
        public static class Value {
            public double OPEN                       = 180; // Degrees
            public double CLOSE                      = 0; // Degrees
        }
    }


}
