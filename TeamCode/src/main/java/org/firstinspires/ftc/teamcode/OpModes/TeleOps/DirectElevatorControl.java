package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.HardwareRobot;
import org.firstinspires.ftc.teamcode.RobotClasses.Elevator;

@TeleOp(name= "DirectElevatorControl", group = "Testing")
@Disabled
public class DirectElevatorControl extends LinearOpMode {

    private HardwareRobot robot = new HardwareRobot();
    private Elevator elevator = new Elevator();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        elevator.init(robot, this);

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.dpad_up) {
                elevator.moveElevator(0.5);
            } else if (gamepad1.dpad_down) {
                elevator.moveElevator(-0.5);
            } else {
                elevator.moveElevator(0);
            }

            if (!gamepad1.left_bumper && gamepad1.a) elevator.setRatchetLock(true);
            if (!gamepad1.left_bumper && gamepad1.b) elevator.setRatchetLock(false);
            if (gamepad1.x) elevator.closeClaw();
            if (gamepad1.y) elevator.openClaw();
            if(gamepad1.left_bumper && gamepad1.a) elevator.moveElevatorToHookHeight();
            if(gamepad1.left_bumper && gamepad1.b) elevator.moveElevatorDown();
        }
    }
}
