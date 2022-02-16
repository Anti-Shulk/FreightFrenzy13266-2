package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.CRServo;

@Config
public class Constants {
    // TODO: remove all statics except for utility classes so that we can have folders
    public static ArmConstants armConstants;
    public static CameraServoConstants cameraServoConstants;
    public static CarouselConstants carouselConstants;
    public static DriveConstants driveConstants;
    public static TurretConstants turretConstants;
    public static GrabberConstants grabberConstants;
    public static IntakeConstants intakeConstants;
    public static TrapdoorConstants trapdoorConstants;
    


    //
    public static class ArmConstants {
        public static Hardware hardware = new Hardware();
        public static Controller controller = new Controller();
        public static Value value = new Value();

        public static class Hardware {
            public String ID            = "armMotor";
            public boolean REVERSED     = true;
            public double
                    RPM           = 84,
                    CPR           = 1993;
        }

        public static class Controller {
            public double
                    TOLERANCE     = 10,
                    KP            = 5,
                    POWER         = 0.60,
                    LOW_POWER     = 0.1;
        }
        public static class Value {
            public double
                    AUTO_HIGH = 55,
                    AUTO_MID = 40,
                    AUTO_LOW = 8,
                    HIGH          = 80, // Degrees
                    MID           = 0, // Degrees
                    LOW           = 0, // Degrees
//                    INTAKE        = 0, // Degrees
                    SHARED        = 0, // Degrees
                    INITIAL = 10,
                    WONT_HIT_SIDES = 40;
        }
    }

    public static class CameraServoConstants {
        public static Hardware hardware = new Hardware();
            public static Value value = new Value();

        public static class Hardware {
            public boolean REVERSED = false;

        }
        public static class Value {
            public double
                    BLUE_WAREHOUSE             = 1, // Degrees
                    BLUE_CAROUSEL              = 1, // Degrees
                    RED_WAREHOUSE              = 1, // Degrees
                    RED_CAROUSEL               = 1; // Degrees
        }
    }

    public static class CarouselConstants {
        public static Spin spin;
        public static Lift lift;

        public static class Spin {
            public static Hardware hardware = new Hardware();
            public static Controller controller = new Controller();
            public static Value value = new Value();

            public static class Hardware {
                public String ID = "carouselSpinMotor";
                public boolean REVERSED = false;
                public double
                        RPM = 435,
                        CPR = 384.5;
            }
            public static class Controller {
                public double
                        TOLERANCE     = 10,
                        KP            = 5,
                        POWER         = 0.6;
            }
            public static class Value {
                public double SPEED                = 1; // power
            }


        }
        public static class Lift {
            public static Hardware hardware = new Hardware();
            public static Value value = new Value();

            public static class Hardware {
                public String ID = "carouselLiftServo";
                public boolean REVERSED = false;
                public double
                        MIN_ANGLE = 0,
                        MAX_ANGLE = 270;
            }
            public static class Value {
                public double
                        UP                   = 42, // Degrees
                        DOWN                 = 146; // Degrees
            }


        }
    }

//    public static class DriveConstantsBad {
//        public static LeftFront leftFront;
//        public static LeftRear leftRear;
//        public static RightFront rightFront;
//        public static RightRear rightRear;
//
//        public static class LeftFront {
//            public static Hardware hardware = new Hardware();
//
//            public static class Hardware {
//                public String ID            = "leftFront";
//                public boolean REVERSED     = false;
//                public double RPM           = 84;
//                public double CPR           = 1993;
//            }
//        }
//        public static class LeftRear {
//            public static Hardware hardware = new Hardware();
//
//            public static class Hardware {
//                public String ID            = "leftRear";
//                public boolean REVERSED     = false;
//                public double RPM           = 84;
//                public double CPR           = 1993;
//            }
//        }
//        public static class RightFront {
//            public static Hardware hardware = new Hardware();
//
//            public static class Hardware {
//                public String ID            = "rightFront";
//                public boolean REVERSED     = false;
//                public double RPM           = 84;
//                public double CPR           = 1993;
//            }
//        }
//        public static class RightRear {
//            public static Hardware hardware = new Hardware();
//
//            public static class Hardware {
//                public String ID            = "rightRear";
//                public boolean REVERSED     = false;
//                public double RPM           = 84;
//                public double CPR           = 1993;
//            }
//        }
//    }


    public static class GrabberConstants {
        public static Grip grip;
        public static Lift lift;

        public static class Grip {
            public static Hardware hardware = new Hardware();
            public static Value value = new Value();
            public static class Hardware {
                public String ID = "gripperGripServo";
                public boolean REVERSED = false;
                public double
                        MIN_ANGLE     = 0,
                        MAX_ANGLE     = 270;

            }
            public static class Value {
                public double
                        OPEN = 270, // Degrees
                        CLOSE = 193; // Degrees
            }

        }
        public static class Lift {
            public static Hardware hardware = new Hardware();
            public static Value value = new Value();
            public static class Hardware {
                public String ID = "gripperLiftServo";
                public boolean REVERSED = false;
                public double
                        MIN_ANGLE     = 0,
                        MAX_ANGLE     = 270;

            }
            public static class Value {
                public double
                        HIGH = 189, // Degrees
                        MID = 189, // Degrees
                        LOW = 162, // Degrees
                        DOWN = 0; // Degrees
                }

        }
    }
    public static class IntakeConstants {
        public static Hardware hardware = new Hardware();
        public static Value value = new Value();
        public static Controller controller = new Controller();

        public static class Hardware {
            public String ID            = "intakeMotor";
            public boolean REVERSED     = false;
            public double
                    RPM           = 435,
                    CPR           = 1993; // figure this out
        }

        public static class Controller {
            public double
                    TOLERANCE     = 10,
                    KP            = 5,
                    POWER         = 0.6,
                    INIT_POWER    = 0;
        }
        public static class Value {
            public double INTAKE_SPEED   = 1; // % of Power
        }
    }

    public static class TurretConstants {
        public static Hardware hardware = new Hardware();
        public static Value value = new Value();
        public static Controller controller = new Controller();

        public static class Hardware {
            public String ID               = "turretMotor";
            public boolean REVERSED        = false;
            public double
                    RPM              = 80,
                    CPR              = 1425.1 * 2;
        }

        public static class Controller {
            public double
                    TOLERANCE     = 10,
                    KP            = 5,
                    POWER         = 0.4,
                    RANGE         = 360; // Degrees

        }
        public static class Value {
            public double
                    FORWARD = 180, // Degrees
                    LEFT    = -90, // Degrees
                    RIGHT   = 90, // Degrees
                    RETURN  = 0; // Degrees

            public int INITIAL_POSITION = 0; // Degrees
        }
    }







    public static class TrapdoorConstants {
        public static Hardware hardware = new Hardware();
        public static Value value = new Value();

        public static class Hardware {
            public String ID = "trapdoorServo";
            public double MIN_ANGLE = 0;
            public double MAX_ANGLE = 270;
            public boolean REVERSED = true;
        }
        public static class Value {
            public double OPEN                       = 83; // Degrees
            public double CLOSE                      = 30; // Degrees
        }
    }


}
