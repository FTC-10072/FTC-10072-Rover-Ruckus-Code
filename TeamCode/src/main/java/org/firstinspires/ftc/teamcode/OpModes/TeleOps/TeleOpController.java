package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Arm;
import org.firstinspires.ftc.teamcode.RobotClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotClasses.Elevator;
import org.firstinspires.ftc.teamcode.RobotClasses.HardwareRobot;

@TeleOp(name = "TeleOp Controller", group = "TeleOp")
public class TeleOpController extends LinearOpMode {

    private HardwareRobot robot = new HardwareRobot();
    private DriveTrain driveTrain = new DriveTrain();
    private Elevator elevator = new Elevator();
    private Arm arm = new Arm();

    /*
    TASK          | CONTROLLER | WHERE
    drive train   | 1          | joysticks
    intake motion | 1          | dpad
    elevator      | 2          | dpad
    claw          | 2          | y
    ratchet       | 2          | b
    intake spin   | 2          | bumpers
    intake stop   | 2          | triggers
    move arm up   | 2          | x
    move arm down | 2          | a

     */

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        driveTrain.init(robot, this);
        elevator.init(robot, this);
        arm.init(robot, this);

        double leftStickY;
        double rightStickX;
        double elevatorPower;

        elevator.setRatchetLock(false);
        boolean ratchetLocked = false;

        int armloc;


        waitForStart();

        while(opModeIsActive()){
            leftStickY = -gamepad1.left_stick_y;
            rightStickX = gamepad1.right_stick_x;

            // drive train
            driveTrain.arcadeDrive(leftStickY, rightStickX);

           /* if(gamepad1.left_bumper){
                robot.sweeperServo.setPosition(1.0);
            }
            else if(gamepad1.right_bumper){
                robot.sweeperServo.setPosition(0.3);
            }*/

            // elevator
            if(gamepad2.dpad_up){
                //if (elevator.getElevatorPosition() <= elevator.getHookTarget()){
                    elevator.moveElevator(0.5);
                //}
                /*else{
                    telemetry.addData("elevator is at max", elevator.getElevatorPosition() >= elevator.getHookTarget());


                }*/

            }
            if (gamepad1.a){
                arm.spinIntake(0);
            }
            else if (gamepad1.b){
                arm.spinIntake(1);
            }
            else if(gamepad1.x){
                arm.spinIntake(2);
            }
            if (gamepad1.right_trigger > .15){
                arm.openBasket();
            }
            else if (gamepad1.left_trigger > .15){
                arm.closeBasket();
            }
            else if(gamepad2.dpad_down){
                if(ratchetLocked) {
                    elevator.moveElevator(-0.5);
                }
                else{
                    telemetry.addData("Lock the ratchet", !ratchetLocked);
                }
            }
            else{
                if(!ratchetLocked) {
                    elevator.moveElevator(0.0);
                }
                else{
                    telemetry.addData("unlock the ratchet", true);
                }
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
            //arm
            /*if (gamepad2.left_bumper){
                arm.stopSpinIntake();
                arm.spinIntake();
            }
            else if (gamepad2.right_bumper){
                arm.stopSpinIntake();
                arm.spinIntakeBack();
            }
            else if (gamepad2.right_bumper && gamepad2.left_bumper){
                arm.stopSpinIntake();
            }*/
            // catch servo
            /*if (gamepad2.left_bumper){
                arm.closeClaw();
            }
            else if (gamepad2.right_bumper){
                arm.openClaw();
            }

            else{
                arm.keepClaw();
            }*/
            /*if (gamepad2.right_bumper)
                arm.spinIntake();
            if (gamepad2.left_bumper)
                arm.spinIntakeBack();
            if(gamepad2.left_bumper && gamepad2.right_bumper)
                arm.stopspinIntake();
            //move arm
            */
            if (gamepad2.left_trigger > .15){
                if(robot.armMotor.getCurrentPosition() > 0)
                    arm.moveArm(.5);

            }
            else if (gamepad2.right_trigger > .15){
                if(robot.armMotor.getCurrentPosition() < 4350)
                    arm.moveArm(-.5);
            }
            else{
                arm.moveArm(0);
            }
            // raise arm
            if (gamepad2.left_stick_y >= .15){
                if (robot.riseMotor.getCurrentPosition() > -904)
                    arm.raiseArm(-.25);
                else
                    arm.raiseArm(0.0);

            }
            else if (gamepad2.left_stick_y <= -.15){
                if (robot.riseMotor.getCurrentPosition() < 394)
                    arm.raiseArm(.25);
                else
                    arm.raiseArm(0.0);
            }
            else{
                arm.raiseArm(0);
            }
            // telemetry
            telemetry.addData("Path0",  "Starting at %7d :%7d",
                    robot.armMotor.getCurrentPosition(),
                    robot.riseMotor.getCurrentPosition());
            telemetry.update();
            telemetry.addData("Ratchet Locked", ratchetLocked);
            telemetry.update();
        }
    }
}
