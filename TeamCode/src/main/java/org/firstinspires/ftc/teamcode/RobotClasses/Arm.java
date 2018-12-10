package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    LinearOpMode currentOpMode;
    DcMotor armMotor;
    Servo extentionServo;

    public void init(HardwareRobot robot, LinearOpMode opMode){
        currentOpMode = opMode;
    }
}
