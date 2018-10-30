package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotClasses.HardwareRobot;

@TeleOp(name = "ServoTest", group = "Testing")
public class ServoTest extends LinearOpMode {

    public Servo intakeMotion;

    public void runOpMode(){
        intakeMotion = hardwareMap.get(Servo.class, "intake_motion");
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.a) {
                intakeMotion.setPosition(0.0);
            } else if (gamepad1.b) {
                intakeMotion.setPosition(0.25);
            }else if(gamepad1.y) {
                intakeMotion.setPosition(0.75);
            }else if(gamepad1.x){
                intakeMotion.setPosition(1.00);
            } else {
                intakeMotion.setPosition(0.5);
            }

        }
    }
}
