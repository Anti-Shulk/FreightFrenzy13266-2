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
    public static ColorRangeSensorConstants colorRangeSensorConstants;
    public static LiftServoConstants liftServoConstants;
    public static LiftMotorConstants liftMotorConstants;
    


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
                    TOLERANCE     = 8,
                    POSITION_TOLERANCE = 8,
                    KP            = 4,
                    TELEOP_POWER = 1,
                    DROP_POWER = 1,
                    AUTO_POWER     = 1;
        }













        public static class Value {
            public double
                    AUTO_HIGH = 70,
                    AUTO_MID = 24,
                    AUTO_LOW = 3,
                    HIGH          = 70, // Degrees
                    MID           = 30, // Degrees
                    LOW           = 0, // Degrees
//                    INTAKE        = 0, // Degrees
                    SHARED        = 0, // Degrees
                    INITIAL = 0,
                    SUS_POSITION = 67,
                    SHARED_UP = 0;
            public enum Height {
                HIGH, MID, LOW, AUTO_HIGH, AUTO_MID, AUTO_LOW, SHARED, SHARED_UP
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
                public double FAST_SPEED = 3000; // power
                public double SLOW_SPEED = 2000;
                public double AUTO_SPEED = 1300;
                public int SLOW_TO_FAST_WAIT = 2000;
                public int HOW_LONG_IT_STAYS_FAST = 1500;
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
                        UP                   = 170, // Degrees
                        DOWN                 = 75; // Degrees
            }


        }
    }
    public static class LiftMotorConstants {
        public static Hardware hardware = new Hardware();
        public static Controller controller = new Controller();
        public static Position position = new Position();
        public static Speed speed = new Speed();

        public static class Hardware {
            public String LEFT_ID = "leftLiftMotor";
            public String RIGHT_ID = "rightLiftMotor";
            public boolean LEFT_REVERSED = false;
            public boolean RIGHT_REVERSED = true;
            public double
                    RPM           = 435,
                    CPR           = 384.539792388;
        }

        public static class Controller {
            public double
                    TOLERANCE     = 8,
                    POSITION_TOLERANCE = 8,
                    KP            = 4,
                    kI            = 0,
                    kD            = 0,
                    kF            = 0;
        }
        public static class Position {
            public double
                    HIGH          = 1000, // Degrees
                    MID           = 600, // Degrees
                    LOW           = 100, // Degrees
                    INITIAL = 0,
                    SHARED_HIGH = 400,
                    SHARED_LOW = 100,
                    CAP_HIGH = 1210,
                    CAP_LOW = 1210,
                    CAP_PICKUP = 350,
                    MAX_POSITION = 1240,
                    MIN_POSITION = 0;
        }
        public static class Speed {
            public double
                    NORMAL_SPEED          = 1,
                    INITIAL_SPEED         = 1,
                    SPEED_DEGREES_CHANGE          = 10;

        }
    }

    public static class LiftServoConstants {
        public static Hardware hardware = new Hardware();
        public static Position position = new Position();
        public static Speed speed = new Speed();

        public static class Hardware {
            public String ID            = "liftServo";
            public boolean REVERSED     = true;
        }

        public static class Position {
            public double
                    HIGH          = 80, // Degrees
                    MID           = 80, // Degrees
                    LOW           = 80, // Degrees
                    INITIAL = 270,
                    SHARED_HIGH = 0,
                    SHARED_LOW = 0,
                    CAP_HIGH = 104,
                    CAP_LOW = 76,
                    CAP_PICKUP = 54;
        }
        public static class Speed {
            public double
                    SPEED_DEGREES_CHANGE          = 1;

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
            public boolean REVERSED     = true;
            public double
                    RPM           = 435,
                    CPR           = 8192; // figure this out
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
                    CPR              = 1425.1/*8192*/ * 2;
        }

        public static class Controller {
            public double
                    PID_TOLERANCE = 3,
                    AT_TARGET_POSITION_TOLERANCE = 2,
                    KP            = 15,
                    POWER         = 1,
                    RANGE         = 360; // Degrees

        }
        public static class Value {
            public double
                    FORWARD = 180, // Degrees
                    NEGATIVE_FORWARD = -180, // Degrees
                    LEFT    = -90, // Degrees
                    RIGHT   = 90, // Degrees
                    RETURN  = -2,
                    SHARED_LEFT = -90,
                    SHARED_RIGHT = 90; // Degrees

//            public double INITIAL_POSITION = 0; // Degrees
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
            public double OPEN                       = 60; // Degrees
            public double CLOSE                      = 17; // Degrees
            public double INTAKE = 52;
            public double CAP_HIGH = 60;
            public double CAP_LOW = 60;
            public double CAP_PICKUP = 52;
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
                    DOWN = 47, // Degrees
                    HIGH = 200,
                    MID = 150,
                    LOW = 130,
                    SHARED = 150,
                    SHARED_UP = 170;
        }

    }

    public static class ColorRangeSensorConstants {
        public static Hardware hardware = new Hardware();
        public static Value value = new Value();
        public static class Hardware {
            public String ID = "colorRangeSensor";
        }
        public static class Value {
            public double
                    DISTANCE_THRESHOLD = 53; // mm
        }

    }

}
