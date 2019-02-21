package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareRobot {

    public DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
    public DcMotor elevatorMotor, armMotor,riseMotor, intakeMotor;
    public Servo ratchetServo, clawServo, catchServo, blockServo;

    public Servo markerServo;
    public BNO055IMU imu;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public HardwareRobot(){}

    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;

        // setup motors
        leftFrontMotor     = hwMap.get(DcMotor.class,"left_motor_1");
        leftBackMotor      = hwMap.get(DcMotor.class,"left_motor_2");
        rightFrontMotor    = hwMap.get(DcMotor.class,"right_motor_1");
        rightBackMotor     = hwMap.get(DcMotor.class,"right_motor_2");
        elevatorMotor      = hwMap.get(DcMotor.class,"elevator_motor");
        armMotor           = hwMap.get(DcMotor.class,"arm_motor");
        riseMotor          = hwMap.get(DcMotor.class,"rise_motor");
        intakeMotor        = hwMap.get(DcMotor.class, "intake_motor");


        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        // positive is clockwise/up, negative is anticlockwise/down
        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        riseMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        riseMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        elevatorMotor.setPower(0);
        armMotor.setPower(0);
        riseMotor.setPower(0);
        intakeMotor.setPower(0);

        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        riseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        riseMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        // set up servos
        ratchetServo = hwMap.get(Servo.class,"ratchet_servo");
        clawServo    = hwMap.get(Servo.class,"claw_servo");
        catchServo   = hwMap.get(Servo.class, "catch_servo");
        blockServo   = hwMap.get(Servo.class, "block_servo");


        // temporary?
        //sweeperServo = hwMap.get(Servo.class,"sweeper_servo");
        markerServo  = hwMap.get(Servo.class,"marker_servo");

        ratchetServo.setPosition(0.5);
        clawServo.setPosition(0.0);
        catchServo.setPosition(.95);

        //sweeperServo.setPosition(1.0);
        markerServo.setPosition(1.0);

        // set up IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu                             = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
}
