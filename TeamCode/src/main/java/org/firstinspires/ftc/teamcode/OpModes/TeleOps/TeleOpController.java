package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotClasses.IntakeOuttake;
import org.firstinspires.ftc.teamcode.RobotClasses.HardwareRobot;

@TeleOp(name = "TeleOp Controller", group = "TeleOp")
public class TeleOpController extends LinearOpMode {

    private HardwareRobot robot = new HardwareRobot();
    private DriveTrain driveTrain = new DriveTrain();
    private IntakeOuttake elevator = new IntakeOuttake();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        driveTrain.init(robot, this);
        elevator.init(robot, this);

        double leftStickY;
        double rightStickX;
        double elevatorPower;

        elevator.setRatchetLock(false);

        waitForStart();

        while(opModeIsActive()){
            leftStickY = -gamepad1.left_stick_y;
            rightStickX = gamepad1.right_stick_x;

            driveTrain.arcadeDrive(leftStickY, rightStickX);

            if(gamepad1.left_bumper){
                elevatorPower = 0.7;
            }
            else if(gamepad1.right_bumper){
                elevatorPower = -0.7;
            }
            else{
                elevatorPower = 0.0;
            }

            elevator.moveElevator(elevatorPower);
        }
    }
}
