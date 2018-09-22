package org.firstinspires.ftc.teamcode.RobotClasses;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class DriveTrain {

    private LinearOpMode currentOpMode;
    private DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;

    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle;

    private static final double DRIVE_P = 0.5, TURN_P = 0.5;
    private static final double MAX_DRIVE_SPEED = 0.7;
    private static final double GAIN = 0.1;
    private static final double DEADBAND = 0.15;

    private static final int COUNTS_PER_REV = 1680; // count / rev
    private static final double WHEEL_DIAMETER = 4.0; // inches
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.141592; // distance / rev
    private static final double COUNTS_PER_INCH = COUNTS_PER_REV / WHEEL_CIRCUMFERENCE;

    public DriveTrain(){}

    public void init(HardwareRobot robot, LinearOpMode opMode){
        currentOpMode = opMode;
        leftFrontMotor = robot.leftFrontMotor;
        leftBackMotor = robot.leftBackMotor;
        rightFrontMotor = robot.rightFrontMotor;
        rightBackMotor = robot.rightBackMotor;

        opMode.telemetry.addData("Mode", "calibrating...");
        opMode.telemetry.update();
        imu = robot.imu;
        while(currentOpMode.opModeIsActive() && !imu.isGyroCalibrated()){
            currentOpMode.sleep(50);
        }
        opMode.telemetry.addData("Mode", "finished calibrating.");
        opMode.telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        opMode.telemetry.update();

        resetAngle();
    }

    // drive to specified distance with specified precision
    @SuppressLint("Assert")
    public boolean driveToDistance(double targetDistance, double precision, double timeout){
        // assert values
        assert precision > 0;
        assert timeout > 0;
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // reset position and velocity
        imu.startAccelerationIntegration(new Position(), new Velocity(), 50);
        resetAngle();
        // loop values
        double currentDistance = imu.getPosition().toUnit(DistanceUnit.INCH).x;
        double diff = targetDistance - currentDistance;
        double power;
        // reset time
        ElapsedTime time = new ElapsedTime();
        // loop until diff is within precision or timeout
        while(currentOpMode.opModeIsActive() && time.seconds() < timeout
                && -precision < diff && diff < precision ){
            // set power
            power = diff * DRIVE_P;
            setLeftPower(power, MAX_DRIVE_SPEED, checkDirection());
            setRightPower(power, MAX_DRIVE_SPEED);
            // update values
            currentDistance = imu.getPosition().toUnit(DistanceUnit.INCH).x;
            diff = targetDistance - currentDistance;
            // sleep
            currentOpMode.sleep(50);
        }
        // check if finished
        if(time.seconds() > timeout) return false;
        // stop motors
        setLeftPower(0.0, 0.0);
        setRightPower(0.0, 0.0);
        imu.stopAccelerationIntegration();
        // return that it ended under timeout
        return true;
    }

    // turn to specific degree. for use in Auto
    // positive is to the left, negative is to the right
    @SuppressLint("Assert")
    public boolean turnToDegree(double targetAngle, double precision, double timeout){
        // assert values
        assert precision > 0;
        assert timeout > 0;
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // make sure targetAngle is bounded
        if(targetAngle > 180.0) targetAngle = 179.9;
        if(targetAngle < -180.0) targetAngle = -179.9;
        // reset angle
        resetAngle();
        // loop values
        double currentAngle = getAngle();
        double diff = currentAngle - targetAngle;
        double power;
        // reset time
        ElapsedTime time = new ElapsedTime();
        while(currentOpMode.opModeIsActive() && time.seconds() < timeout
                && -precision < diff && diff < precision){
            // set power
            power = diff * TURN_P;
            setLeftPower(power, MAX_DRIVE_SPEED);
            setRightPower(-power, MAX_DRIVE_SPEED);
            // update values
            currentAngle = getAngle();
            diff = currentAngle - targetAngle;
            // sleep
            currentOpMode.sleep(50);
        }
        // check if finished
        if(time.seconds() > timeout) return false;
        // stop motors
        setLeftPower(0.0, 0.0);
        setRightPower(0.0, 0.0);
        resetAngle();
        return true;
    }


    public void arcadeDrive(double move, double turn){
        move = boundValue(move);
        move = deadband(move, DEADBAND);
        turn = boundValue(turn);
        turn = deadband(turn, DEADBAND);

        // square input while keeping sign
        move *= Math.abs(move);
        turn *= Math.abs(turn);

        double leftPower, rightPower;
        double maxInput = Math.copySign(Math.max(Math.abs(move), Math.abs(turn)), move);

        if(move >= 0){
            if(turn >= 0){
                leftPower = maxInput;
                rightPower = move - turn;
            }else{
                leftPower = move + turn;
                rightPower = maxInput;
            }
        }else{
            if(turn >= 0){
                leftPower = move + turn;
                rightPower = maxInput;
            }else{
                leftPower = maxInput;
                rightPower = move - turn;
            }
        }

        setLeftPower(leftPower, MAX_DRIVE_SPEED);
        setRightPower(rightPower, MAX_DRIVE_SPEED);
    }

    // normalize power and set left motors to that power (with correction)
    private void setLeftPower(double power, double maxPower, double correction){
        power = boundValue(power) * maxPower;
        leftFrontMotor.setPower(power + correction);
        leftBackMotor.setPower(power + correction);
    }

    // normalize power and set left motors
    private void setLeftPower(double power, double maxPower){
        power = boundValue(power) * maxPower;
        leftFrontMotor.setPower(power);
        leftBackMotor.setPower(power);
    }

    // normalize power and set right motors to that power
    private void setRightPower(double power, double maxPower){
        power = boundValue(power) * maxPower;
        rightFrontMotor.setPower(power);
        rightBackMotor.setPower(power);
    }

    // bound value to -1.0, 1.0
    private double boundValue(double value){
        if(value > 1.0) return 1.0;
        else if(value < -1.0) return -1.0;
        else return value;
    }

    // return 0 if abs(value) is less than band
    private double deadband(double value, double band){
        if(-band < value && value < band) return 0;
        return value;
    }

    // set all motor modes
    private void setMode(DcMotor.RunMode mode){
        leftFrontMotor.setMode(mode);
        leftBackMotor.setMode(mode);
        rightFrontMotor.setMode(mode);
        rightBackMotor.setMode(mode);
    }

    // reset the gyro angle
    private void resetAngle(){
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    // get the current gyro angle
    // positive is to the left, negative is to the right
    private double getAngle(){
        Orientation newAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = newAngles.firstAngle - lastAngles.firstAngle;
        if(deltaAngle < -180){
            deltaAngle += 360;
        }
        else if(deltaAngle > 180){
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;
        lastAngles = newAngles;

        return globalAngle;
    }

    // get correction factor for driving forward
    private double checkDirection(){
        double angle = getAngle();
        return angle * GAIN;
    }
}
