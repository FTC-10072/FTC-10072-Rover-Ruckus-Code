package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    private LinearOpMode currentOpMode;

    private Servo gripperServo;
    //private Servo catchServo;



    public void init(HardwareRobot robot, LinearOpMode opMode){

        currentOpMode = opMode;
        gripperServo = robot.gripperServo;
        gripperServo.setPosition(1);




    }


    public void dropgripper(){
        gripperServo.setPosition(.5);
    }
    public void raisegripper(){
        gripperServo.setPosition(1);
    }







}
