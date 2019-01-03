package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotClasses.Elevator;
import org.firstinspires.ftc.teamcode.RobotClasses.HardwareRobot;

@TeleOp(name = "TeleOp Controller", group = "TeleOp")
public class TeleOpController extends LinearOpMode {

    private HardwareRobot robot = new HardwareRobot();
    private DriveTrain driveTrain = new DriveTrain();
    private Elevator elevator = new Elevator();

    /*
    TASK          | CONTROLLER | WHERE
    drive train   | 1          | joysticks
    intake motion | 1          | dpad
    intake spin   | 1          | bumpers
    intake flip   | 1          | triggers
    elevator      | 2          | dpad
    outtake flip  | 2          | a / x
    claw          | 2          | y
    ratchet       | 2          | b
     */

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        driveTrain.init(robot, this);
        elevator.init(robot, this);

        double leftStickY;
        double rightStickX;
        double elevatorPower;

        elevator.setRatchetLock(false);
        boolean ratchetLocked = false;

        waitForStart();

        while(opModeIsActive()){
            leftStickY = -gamepad1.left_stick_y;
            rightStickX = gamepad1.right_stick_x;

            // drive train
            driveTrain.arcadeDrive(leftStickY, rightStickX);

            if(gamepad1.left_bumper){
                robot.sweeperServo.setPosition(1.0);
            }
            else if(gamepad1.right_bumper){
                robot.sweeperServo.setPosition(0.3);
            }

            // elevator
            if(gamepad2.dpad_up){
                elevator.moveElevator(0.5);
            }
            else if(gamepad2.dpad_down){
                elevator.moveElevator(-0.5);
            }
            else{
                elevator.moveElevator(0.0);
            }

            // claw
            if(gamepad2.a){
                elevator.closeClaw();
            }
            else if(gamepad2.b){
                elevator.openClaw();
            }

            // ratchet
            if(gamepad2.y){
                elevator.setRatchetLock(true);
                ratchetLocked = true;
            }
            else if(gamepad2.x){
                elevator.setRatchetLock(false);
                ratchetLocked = false;
            }

            // telemetry
            telemetry.addData("Ratchet Locked", ratchetLocked);
            telemetry.update();
        }
    }
}
