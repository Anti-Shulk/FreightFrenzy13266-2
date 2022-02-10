package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public static Arm arm = new Arm();
    public static Carousel carousel = new Carousel();
    public static Turret turret = new Turret();
    public static Gripper gripper = new Gripper();
    public static Trapdoor trapdoor = new Trapdoor();
    public static CameraServo cameraServo = new CameraServo();
    


    // TODO: make it work in degrees
    public static class Arm {
        public static Hardware hardware = new Hardware();
        public static Controller controller = new Controller();

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



        public double HIGH          = 30; // Degrees
        public double MID           = 0; // Degrees
        public double LOW           = 0; // Degrees
        public double INTAKE        = 0; // Degrees

        public int INITIAL_POSITION = 0;

    }

    public static class Carousel {
        public static Spin spin = new Spin();
        public static Lift lift = new Lift();

        public static class Spin {
            public static Hardware hardware = new Hardware();
            public static Controller controller = new Controller();

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

            public double SPEED                = 30; // Degrees per second
        }
        public static class Lift {
            public static Hardware hardware = new Hardware();
            public static class Hardware {
                public String ID = "carouselLiftServo";
                public boolean REVERSED = false;
                public double MIN_ANGLE = 0;
                public double MAX_ANGLE = 270;
            }

            public double UP                   = 90; // Degrees
            public double DOWN                 = 0; // Degrees
        }




    }
    public static class Drive {
        public static LeftFront leftFront = new LeftFront();
        public static LeftRear leftRear = new LeftRear();
        public static RightFront rightFront = new RightFront();
        public static RightRear rightRear = new RightRear();

        public static class LeftFront {
            public static Hardware hardware = new Hardware();

            public static class Hardware {
                public String ID            = "leftFront";
                public boolean REVERSED     = false;
                public double RPM           = 84;
                public double CPR           = 1993;
            }
        }
        public static class LeftRear {
            public static Hardware hardware = new Hardware();

            public static class Hardware {
                public String ID            = "leftRear";
                public boolean REVERSED     = false;
                public double RPM           = 84;
                public double CPR           = 1993;
            }
        }
        public static class RightFront {
            public static Hardware hardware = new Hardware();

            public static class Hardware {
                public String ID            = "rightFront";
                public boolean REVERSED     = false;
                public double RPM           = 84;
                public double CPR           = 1993;
            }
        }
        public static class RightRear {
            public static Hardware hardware = new Hardware();

            public static class Hardware {
                public String ID            = "rightRear";
                public boolean REVERSED     = false;
                public double RPM           = 84;
                public double CPR           = 1993;
            }
        }
        



        public double HIGH          = 30; // Degrees
        public double MID           = 0; // Degrees
        public double LOW           = 0; // Degrees
        public double INTAKE        = 0; // Degrees

        public int INITIAL_POSITION = 0;

    }


    public static class Gripper {
        public static Grip grip = new Grip();
        public static Lift lift = new Lift();

        public static class Grip {
            public static Hardware hardware = new Hardware();
            public static class Hardware {
                public String ID = "gripperServo";
                public double MIN_ANGLE     = 0;
                public double MAX_ANGLE     = 270;
                public boolean REVERSED = false;
            }
            public double OPEN               = 180; // Degrees
            public double CLOSE              = 0; // Degrees

        }
        public static class Lift {
            public static Hardware hardware = new Hardware();
            public static class Hardware {
                public String ID = "gripperLiftServo";
                public double MIN_ANGLE     = 0;
                public double MAX_ANGLE     = 270;
                public boolean REVERSED = false;
            }
            public double UP                 = 90; // Degrees
            public double DOWN               = 0; // Degrees

        }


    }

    public static class Turret {
        public static Hardware hardware = new Hardware();
        public static Controller controller = new Controller();

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

        public double FORWARD = 0; // Degrees
        public double LEFT    = 0; // Degrees
        public double RIGHT   = 0; // Degrees
        public double INTAKE   = 0; // Degrees
    }

    public static class Intake {
        public static Hardware hardware = new Hardware();
        public static Controller controller = new Controller();
        public static class Hardware {
            public String ID = "intakeMotor";
            public boolean REVERSED = false;
            public double RPM = 80;
            public double CPR = 1000;
        }
        public static class Controller {

        }

        public double INTAKE_SPEED   = 1; // % of Power
    }





    public static class Trapdoor {
        public static Hardware hardware = new Hardware();
        public static class Hardware {
            public String ID = "trapdoorServo";
            public double MIN_ANGLE = 0;
            public double MAX_ANGLE = 270;
            public boolean REVERSED = false;
        }

        public double OPEN                       = 180; // Degrees
        public double CLOSE                      = 0; // Degrees
    }

    public static class CameraServo {

        public String ID = "cameraServo";
        public double MIN_ANGLE     = 0;
        public double MAX_ANGLE     = 270;
        public boolean REVERSED = false;

        public double BLUE_WAREHOUSE             = 1; // Degrees
        public double BLUE_CAROUSEL              = 1; // Degrees
        public double RED_WAREHOUSE              = 1; // Degrees
        public double RED_CAROUSEL               = 1; // Degrees
    }
}
