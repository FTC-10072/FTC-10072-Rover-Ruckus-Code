package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.HardwareRobot;

@TeleOp(name = "WheelTesting", group="Testing")
public class WheelTesting extends LinearOpMode {

    private HardwareRobot robot = new HardwareRobot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        robot.leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.dpad_up){rotate360(robot.leftFrontMotor);}
            else{}
            if(gamepad1.dpad_left){ rotate360(robot.leftBackMotor);}
            else{}
            if(gamepad1.dpad_right) {rotate360(robot.rightFrontMotor);}
            else{}
            if(gamepad1.dpad_down) {rotate360(robot.rightBackMotor);}
            else{}
        }
    }

    public void setMode(DcMotor.RunMode mode){
        robot.leftBackMotor.setMode(mode);
        robot.leftFrontMotor.setMode(mode);
        robot.rightBackMotor.setMode(mode);
        robot.rightFrontMotor.setMode(mode);
    }

    public void rotate360(DcMotor motor){
        if(opModeIsActive()) {
            motor.setTargetPosition(motor.getCurrentPosition() + 1680);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(0.5);
            while (opModeIsActive() && motor.isBusy()){}
            motor.setPower(0.0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
