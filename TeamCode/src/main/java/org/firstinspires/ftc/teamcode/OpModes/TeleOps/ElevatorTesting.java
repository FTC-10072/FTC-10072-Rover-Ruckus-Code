package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.HardwareRobot;

@TeleOp(name = "ElevatorTesting", group="Testing")
public class ElevatorTesting extends LinearOpMode {

    private HardwareRobot robot = new HardwareRobot();
    private ElapsedTime runtime = new ElapsedTime();
    private static final int TOP_TARGET = (int)(1120 * 2.375);

    DcMotor motor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        robot.ratchetServo.setPosition(0.0);
        motor = robot.elevatorMotor;
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.7);
        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.dpad_up){
                motor.setTargetPosition(TOP_TARGET);
            } else if(gamepad1.dpad_down){
                motor.setTargetPosition(0);
            }
        }
    }
}
