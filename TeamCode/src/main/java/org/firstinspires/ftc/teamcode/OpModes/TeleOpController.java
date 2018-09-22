package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotClasses.HardwareRobot;

public class TeleOpController extends LinearOpMode {

    private HardwareRobot robot = new HardwareRobot();
    private DriveTrain driveTrain = new DriveTrain();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        driveTrain.init(robot, this);

        double leftStickY;
        double rightStickX;

        waitForStart();

        while(opModeIsActive()){
            leftStickY = gamepad1.left_stick_y;
            rightStickX = gamepad1.right_stick_x;

            driveTrain.arcadeDrive(leftStickY, rightStickX);
        }
    }
}
