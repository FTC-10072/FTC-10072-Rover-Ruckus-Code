package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotClasses.Arm;
import org.firstinspires.ftc.teamcode.RobotClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotClasses.Elevator;
import org.firstinspires.ftc.teamcode.RobotClasses.HardwareRobot;

@TeleOp(name = "ServoTest", group = "TeleOp")
//@Disabled
public class ServoTest extends LinearOpMode {

    //public Servo intakeMotion;

    public void runOpMode() {
        //intakeMotion = hardwareMap.get(Servo.class, "intake_motion");
        Arm arm = new Arm();
        Elevator elevator = new Elevator();
        DriveTrain driveTrain = new DriveTrain();
        double leftStickY;
        double rightStickX;
        boolean lock;

        waitForStart();
        while (opModeIsActive()) {

            leftStickY = -gamepad1.left_stick_y;
            rightStickX = gamepad1.right_stick_x;

            // drive train testing
            //driveTrain.arcadeDrive(leftStickY, rightStickX);
            //Arm test
            if (gamepad1.left_bumper){
                arm.raiseArm(.1);
            }
            else if (gamepad1.right_bumper){
                arm.raiseArm(-.1);
            }
            //arm extension test
            if (gamepad1.right_trigger >= .15){
                arm.moveArm(.1);
            }
            else if (gamepad1.left_trigger >= .15){
                arm.moveArm(.1);
            }
            //elevator test
            if (gamepad1.a){
                elevator.moveElevator(.1);
            }
            else if(gamepad1.b){
                elevator.moveElevator(-.1);
            }
            if (gamepad1.x){
                elevator.setRatchetLock(true);
            }
            if(gamepad1.y){
                elevator.setRatchetLock(false);
            }

            /*if (gamepad1.a) {
                intakeMotion.setPosition(0.0);
            } else if (gamepad1.b) {
                intakeMotion.setPosition(0.5);
            }else if(gamepad1.y) {
                intakeMotion.setPosition(1.5);
            }else if(gamepad1.x){
                intakeMotion.setPosition(2.00);
            } else {
                intakeMotion.setPosition(1.00);
            }

        }*/
        }
    }
}
