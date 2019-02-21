package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotClasses.HardwareRobot;

@Autonomous(name = "Just Move Forward Please", group = "Auto")
public class JustMoveForwardPlease extends LinearOpMode {
    private HardwareRobot robot = new HardwareRobot();
    private DriveTrain driveTrain = new DriveTrain();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        driveTrain.init(robot, this);

        waitForStart();

        //driveTrain.driveToDistance(24*Math.sqrt(2.0),7);
        driveTrain.turnToDegree(90,.1,3);
        driveTrain.stop();
    }
}
