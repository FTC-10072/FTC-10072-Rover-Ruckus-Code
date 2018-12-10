package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotClasses.Elevator;
import org.firstinspires.ftc.teamcode.RobotClasses.HardwareRobot;
import org.firstinspires.ftc.teamcode.RobotClasses.PhoneVision;

@Autonomous(name="CraterAuto", group="Autonomous")
public class CraterAuto extends LinearOpMode {
    HardwareRobot robot = new HardwareRobot();
    DriveTrain driveTrain = new DriveTrain();
    Elevator elevator = new Elevator();
    PhoneVision vision;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        driveTrain.init(robot, this);
        elevator.init(robot, this);
        elevator.closeClaw();
        elevator.setRatchetLock(true);
        elevator.moveElevatorDown();
        vision = new PhoneVision(this);

        waitForStart();

        // disengage from the hook
        /*
        elevator.moveElevatorToHookHeight();
        sleep(3000);
        elevator.openClaw();
        elevator.moveElevatorDown();
        */

        // move to mineral
        double turnAngle = 23.565;
        double shortDistance = Math.sqrt(8.0) * 12 - 2;
        double longDistance = Math.sqrt(10.0) * 12 - 3;
        PhoneVision.Direction direction = vision.getDirection();
        if(direction == PhoneVision.Direction.LEFT){
            driveTrain.turnToDegree(turnAngle, 0.1, 2.0);
            driveTrain.driveToDistance(longDistance, 3.5);
            driveTrain.turnToDegree(-2*turnAngle, 0.1, 2.0);
            driveTrain.driveToDistance(longDistance/4, 3.5);
        }
        else if(direction == PhoneVision.Direction.RIGHT){
            driveTrain.turnToDegree(-turnAngle, 0.1, 2.0);
            driveTrain.driveToDistance(longDistance, 3.5);
            driveTrain.turnToDegree(2*turnAngle, 0.1, 2.0);
            driveTrain.driveToDistance(longDistance/4, 3.5);
        }
        else{
            driveTrain.driveToDistance(shortDistance, 3.0);
            driveTrain.driveToDistance(shortDistance/4, 3.0);
        }
        driveTrain.stop();
    }
}
