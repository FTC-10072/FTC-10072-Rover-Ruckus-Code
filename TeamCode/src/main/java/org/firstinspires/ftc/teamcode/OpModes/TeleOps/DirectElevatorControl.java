package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.HardwareRobot;
import org.firstinspires.ftc.teamcode.RobotClasses.IntakeOuttake;

@TeleOp(name= "DirectElevatorControl", group = "Testing")
public class DirectElevatorControl extends LinearOpMode {

    private HardwareRobot robot = new HardwareRobot();
    private IntakeOuttake intakeOuttake = new IntakeOuttake();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        intakeOuttake.init(robot, this);

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.dpad_up) {
                intakeOuttake.moveElevator(0.5);
            } else if (gamepad1.dpad_down) {
                intakeOuttake.moveElevator(-0.5);
            } else {
                intakeOuttake.moveElevator(0);
            }

            if (!gamepad1.left_bumper && gamepad1.a) intakeOuttake.setRatchetLock(true);
            if (!gamepad1.left_bumper && gamepad1.b) intakeOuttake.setRatchetLock(false);
            if (gamepad1.x) intakeOuttake.closeClaw();
            if (gamepad1.y) intakeOuttake.openClaw();
            if(gamepad1.left_bumper && gamepad1.a) intakeOuttake.moveElevatorUp();
            if(gamepad1.left_bumper && gamepad1.b) intakeOuttake.moveElevatorDown();
        }
    }
}
