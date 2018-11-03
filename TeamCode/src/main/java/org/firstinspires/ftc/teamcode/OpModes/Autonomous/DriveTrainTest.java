package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotClasses.HardwareRobot;

@Autonomous(name = "DriveTrain Test", group = "Testing")
//@Disabled
public class DriveTrainTest extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot();
    DriveTrain driveTrain = new DriveTrain();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        driveTrain.init(robot, this);
        waitForStart();

        driveTrain.driveToDistance(36.0, 6.0);
        //driveTrain.turnToDegree(-90, 1.0, 10);
        //driveTrain.turnToDegree(90, 1.0, 10);
        driveTrain.driveToDistance(-36.0, 6.0);
        driveTrain.stop();
    }
}
