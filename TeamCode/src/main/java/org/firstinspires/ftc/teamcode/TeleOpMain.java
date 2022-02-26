package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

import java.security.PrivateKey;

@TeleOp(name = "christan")
public class TeleOpMain extends OpMode {
    private DcMotor back_Right = null;
    private DcMotor back_Left = null;
    private DcMotor front_Right = null;
    private DcMotor front_Left = null;
    private DcMotorEx turret_Motor = null;
    private DcMotorEx arm_Motor = null;
    private DcMotor intake_Motor = null;
    private Servo carousel_lifter = null;
    private CRServo carousel_spinner = null;
    private Servo flipper_servo;
    private Servo cap_mech;
    private Servo preload_clamp;
    double TicksPerRev = 1425.1 * 2;
    double TicksPerRevArm = 1993;
    public static double ARM_HIGH_POS = -2000;
    public static double ARM_LOW_POS = -120;
    public static double TURRET_HIGH_POS = -4000;
    public static double TURRET_LEFT_POS = -1000;
    public static double TURRET_RIGHT_POS = -500;
    public static double Carosuel_Up = 10;
    public static double Carosuel_Down = 10;
    public static double flipper_open = 10;
    public static double flipper_close = 10;
    public static double capmech_down = 10;
    public static double capmech_Up = 10;
    public static double gripper_close = 10;
    public static double gripper_open = 10;



    @Override
    public void init() {
        back_Left = hardwareMap.get(DcMotor.class, "leftRear");
        back_Right = hardwareMap.get(DcMotor.class, "rightRear");
        front_Left = hardwareMap.get(DcMotor.class, "leftFront");
        front_Right = hardwareMap.get(DcMotor.class, "rightFront");
        front_Left.setDirection(DcMotorSimple.Direction.FORWARD);
        front_Right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_Right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_Left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_Motor=hardwareMap.get(DcMotor.class, "intakeMotor");
        intake_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        intake_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret_Motor = hardwareMap.get(DcMotorEx.class,"turretMotor");
        turret_Motor.setPositionPIDFCoefficients(3);
        turret_Motor.setTargetPositionTolerance(10);

        turret_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        turret_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret_Motor.setTargetPosition(0);
        turret_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_Motor = hardwareMap.get(DcMotorEx.class, "armMotor");
        arm_Motor.setPositionPIDFCoefficients(5);
        arm_Motor.setTargetPositionTolerance(10);
        arm_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        arm_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_Motor.setTargetPosition(0);
        arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addLine("lol if you are reading this you are also gay");
        carousel_lifter = hardwareMap.get(Servo.class, "carouselLifter");
        carousel_spinner = hardwareMap.get(CRServo.class, "carouselSpinner");
        flipper_servo = hardwareMap.get(Servo.class, "trapDoor");
        cap_mech = hardwareMap.get(Servo.class,"capServo");
        preload_clamp = hardwareMap.get(Servo.class, "clampServo");
        carousel_lifter.setDirection(Servo.Direction.FORWARD);
        cap_mech.setDirection(Servo.Direction.FORWARD);
        preload_clamp.setDirection(Servo.Direction.FORWARD);



    }

    @Override
    public void loop() {
        double drive = gamepad1.left_stick_y;
        double turn= -gamepad1.right_stick_x;
        double strafe = -gamepad1.left_stick_x;
        double speedMultiplier;
        telemetry.addData("armpos", arm_Motor.getCurrentPosition());
        telemetry.addData("turretpos", turret_Motor.getCurrentPosition());




        if (gamepad1.left_bumper) speedMultiplier = .3;
        else if (gamepad1.right_bumper) speedMultiplier = 1;
        else speedMultiplier = 1;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(drive) + Math.abs(turn) + Math.abs(strafe), 1);

        back_Left.setPower(((drive + turn - strafe) / denominator) * speedMultiplier);
        back_Right.setPower(((drive - turn + strafe) / denominator) * speedMultiplier);
        front_Right.setPower(((drive - turn - strafe) / denominator) * speedMultiplier);
        front_Left.setPower(((drive + turn + strafe) / denominator) * speedMultiplier);
        turret_Motor.setPower(1);
        arm_Motor.setPower(1);

        telemetry.addData("FrontLeft",(drive + turn + strafe) * speedMultiplier);
        telemetry.addData("FrontRight",(drive - turn - strafe) * speedMultiplier);
        telemetry.addData("BackRight",(drive - turn + strafe) * speedMultiplier);
        telemetry.addData("BackLeft",(drive + turn - strafe) * speedMultiplier);
        //telemetry.addData("armMotor",(arm_Motor.getCurrentPosition()

        //arm code/Turret Code
        if (gamepad2.dpad_up)setArmPosition(ARM_HIGH_POS);
        else if (gamepad2.dpad_up)setTurretPosition(TURRET_HIGH_POS);
        else if (gamepad2.dpad_right)setTurretPosition(TURRET_RIGHT_POS);
        else if (gamepad2.dpad_right)setArmPosition(ARM_LOW_POS);
        else if (gamepad2.dpad_left)setTurretPosition(TURRET_LEFT_POS);
        else if (gamepad2.dpad_left)setArmPosition(ARM_LOW_POS);
        if (gamepad2.right_bumper)flipper_servo.setPosition(flipper_open);
        else flipper_servo.setPosition(flipper_close);


        // intake code
        if (gamepad1.y)intake_Motor.setPower(1);
        else if (gamepad1.a)intake_Motor.setPower(-1);
        else intake_Motor.setPower(0);
        //carousel code
        if (gamepad1.dpad_up)carousel_lifter.setPosition(Carosuel_Up);
        else if (gamepad1.dpad_down)carousel_lifter.setPosition(Carosuel_Down);
         if (gamepad1.dpad_left)carousel_spinner.setPower(1);
         else if (gamepad1.dpad_right)carousel_spinner.setPower(-1);




    }
    private void setTurretPosition(double position) {
        turret_Motor.setTargetPosition((int) (TicksPerRev * position));
    }
    private void setArmPosition (double armpos){
        arm_Motor.setTargetPosition((int) (TicksPerRevArm*armpos));
    }
    private  void setIntake_Motor (double intakepower){
        intake_Motor.setPower(1);
    }

    private void drivetrain(double power){
        back_Right.setPower(power);
        back_Left.setPower(power);
        front_Left.setPower(power);
        front_Right.setPower(power);
    }


}






