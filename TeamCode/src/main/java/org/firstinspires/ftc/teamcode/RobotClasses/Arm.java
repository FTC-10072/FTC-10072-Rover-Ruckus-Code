package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    private LinearOpMode currentOpMode;
    private DcMotor armMotor, riseMotor, intakeMotor;
    private Servo blockServo;
    //private Servo catchServo;



    public void init(HardwareRobot robot, LinearOpMode opMode){

        currentOpMode = opMode;
        armMotor = robot.armMotor;
        riseMotor = robot.riseMotor;
        blockServo = robot.blockServo;
        intakeMotor = robot.intakeMotor;
        //catchServo = robot.catchServo;


        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        riseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        riseMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //catchServo.setPosition(.95);
        blockServo.setPosition(.5);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    //begins to move the arm
    public void moveArm(double speed){
        armMotor.setPower(speed);

    }
    public void spinIntake(int num){
        if ( num == 0)
            intakeMotor.setPower(.5);
        else if (num == 2){
            intakeMotor.setPower(0);
        }
        else
            intakeMotor.setPower(-.5);


    }


   //moves the intake
    /*public void spinIntake(){
        intakeMotor.setPower(.5);
    }
    //spins the intake backwards
    public void spinIntakeBack(){
        intakeMotor.setPower(-.5);

    }
    //stops spinning the intake
    public void stopspinIntake(){
        intakeMotor.setPower(0);

    }*/
    //opens arm Claw

    /*public void openClaw(){
        catchServo.setPosition(.65);
    }

    public void closeClaw(){
        catchServo.setPosition(.5);
    }
    public void startClaw(){
        catchServo.setPosition(.95);
    }
    public double getArmPosition(){
        return armMotor.getCurrentPosition();
    }
    public void keepClaw(){
        catchServo.setPosition(catchServo.getPosition());
    }
    */
    //raises the arm
    public void raiseArm(double speed){

        riseMotor.setPower(speed);

    }
    public void openBasket(){

        blockServo.setPosition(.5);
    }
    public void closeBasket(){
        blockServo.setPosition(0);

    }
    public int getArmPosition(){
        return armMotor.getCurrentPosition();
    }
    public int getRiseArmPosition(){
        return riseMotor.getCurrentPosition();
    }


}
