package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Elevator {
    private LinearOpMode currentOpMode;
    private DcMotor elevatorMotor;
    private Servo ratchetServo, clawServo;
    private boolean ratchetLocked = true;

    private static final double rotationsToHook = 2.475;
    private static final int HOOK_TARGET = (int)(rotationsToHook * 1120);

    public Elevator(){}

    public void init(HardwareRobot robot, LinearOpMode opMode){
        elevatorMotor = robot.elevatorMotor;
        ratchetServo = robot.ratchetServo;
        clawServo = robot.clawServo;
        currentOpMode = opMode;

        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ELEVATOR FUNCTIONS
    public void setRatchetLock(boolean lock){
        ratchetLocked = lock;
        if(lock){
            ratchetServo.setPosition(1.0);
        }else{
            ratchetServo.setPosition(0.5);
        }
    }

    public void openClaw(){
        clawServo.setPosition(1.0);
    }

    public void closeClaw(){
        clawServo.setPosition(0.0);
    }

    public void moveElevatorDown(){
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorMotor.setTargetPosition(0);
        elevatorMotor.setPower(0.5);
    }

    public void moveElevatorToHookHeight(){
        if(!ratchetLocked){
            elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevatorMotor.setTargetPosition(HOOK_TARGET);
            elevatorMotor.setPower(0.5);
        }
    }

    public void moveElevator(double speed){
        if(speed > 0 && !ratchetLocked){
            elevatorMotor.setPower(speed);
        }
        else{
            elevatorMotor.setPower(speed);
        }
    }
}
