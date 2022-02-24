package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    // TODO: remove all statics except for utility classes so that we can have folders
    public static ArmConstants armConstants;
    public static CameraConstants cameraConstants;
    public static CarouselConstants carouselConstants;
    public static DriveConstants driveConstants;
    public static TurretConstants turretConstants;
    public static GrabberConstants grabberConstants;
    public static IntakeConstants intakeConstants;
    public static TrapdoorConstants trapdoorConstants;
    public static IntakeLiftConstants intakeLiftConstants;
    public static BoxConstants boxConstants;
    


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
                    TOLERANCE     = 2,
                    KP            = 15,
                    TELEOP_POWER = 1,
                    DROP_POWER = 1,
                    AUTO_POWER     = 1;
        }
        public static class Value {
            public double
                    AUTO_HIGH = 55,
                    AUTO_MID = 40,
                    AUTO_LOW = 8,
                    HIGH          = 70, // Degrees
                    MID           = 32, // Degrees
                    LOW           = 8, // Degrees
//                    INTAKE        = 0, // Degrees
                    SHARED        = 0, // Degrees
                    INITIAL = 0,
                    SUS_POSITION = 40;
            public enum Height {
                HIGH, MID, LOW, AUTO_HIGH, AUTO_MID, AUTO_LOW, SHARED
            }
        }
    }

    public static class CameraConstants {
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
            public double CAMERA_WAIT_TIME_DOUBLE = 5;
            public long CAMERA_WAIT_TIME = (long) (CAMERA_WAIT_TIME_DOUBLE * 1000);
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
                        POWER         = 1;
            }
            public static class Value {
                public double SPEED                = 0.7; // power
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
                        OPEN = 50, // Degrees
                        CLOSE = 80; // Degrees
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
                        HIGH = 80, // Degrees
                        MID = 80, // Degrees
                        LOW = 103, // Degrees
                        DOWN = 270; // Degrees
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
                    POWER         = 1,
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
                    PID_TOLERANCE = 2,
                    INTAKE_POSITION_TOLERANCE = 2,
                    KP            = 20,
                    POWER         = 1,
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
            public double OPEN                       = 0; // Degrees
            public double CLOSE                      = 35; // Degrees
        }
    }

    public static class IntakeLiftConstants {
        public static Hardware hardware = new Hardware();
        public static Value value = new Value();
        public static class Hardware {
            public String ID = "intakeLiftServo";
            public boolean REVERSED = false;
            public double
                    MIN_ANGLE     = 0,
                    MAX_ANGLE     = 270;

        }
        public static class Value {
            public double
                    UP = 0, // Degrees
                    DOWN = 80; // Degrees
        }

    }

    public static class BoxConstants {
        public static Hardware hardware = new Hardware();
        public static Value value = new Value();
        public static class Hardware {
            public String ID = "boxServo";
            public boolean REVERSED = false;
            public double
                    MIN_ANGLE     = 0,
                    MAX_ANGLE     = 270;

        }
        public static class Value {
            public double
                    DOWN = 210, // Degrees
                    HIGH = 70,
                    MID = 0,
                    LOW = 0;
        }

    }

}
