package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotClasses.HardwareRobot;

@Autonomous(name = "Encoder Test", group = "Testing")
//@Disabled
public class EncoderTest  extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot();
    DriveTrain driveTrain = new DriveTrain();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        driveTrain.init(robot, this);
        waitForStart();

        driveTrain.driveToDistance(12.0, 0.5, 10);
        driveTrain.turnToDegree(-90, 1.0, 10);
        driveTrain.turnToDegree(90, 1.0, 10);
        driveTrain.driveToDistance(-12.0, 0.5, 10);
    }
}
