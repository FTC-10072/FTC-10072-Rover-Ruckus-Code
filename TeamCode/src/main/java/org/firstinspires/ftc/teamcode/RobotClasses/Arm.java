package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    private LinearOpMode currentOpMode;
    private DcMotor armMotor;
    private Servo extentionServo, markerServo;

    public void init(HardwareRobot robot, LinearOpMode opMode){
        currentOpMode = opMode;
    }


}
