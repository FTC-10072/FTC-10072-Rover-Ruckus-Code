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
        double turnAngle = 27.565;
        double shortDistance = Math.sqrt(8.0) * 12 - 2;
        double longDistance = Math.sqrt(10.0) * 12 - 3;
        int depotToCrater = 72;
        elevator.detachFromLander();
        driveTrain.turnToDegree(-15,.5, .5);
        elevator.moveElevatorDown();
        driveTrain.turnToDegree(15,.5, .5);

        driveTrain.driveToDistance(4, .5);
        driveTrain.turnToDegree(90, .1, 1.5);
        driveTrain.stop();
        driveTrain.driveToDistance(-4.25,.75);
        PhoneVision.Direction direction = vision.getDirection();
        //driveTrain.driveToDistance(3.5,.75);
        driveTrain.turnToDegree(-90, .1, 1.5);
        driveTrain.stop();
        if(direction == PhoneVision.Direction.LEFT){
            driveTrain.turnToDegree(turnAngle+15, 0.1, 1.0);
            driveTrain.stop();
            driveTrain.driveToDistance(shortDistance, 2.5);
            //driveTrain.turnToDegree(-2*turnAngle, 0.1, 2.0);
            //driveTrain.driveToDistance(longDistance/4, 3.5);
            driveTrain.turnToDegree(-80, .5, 1.5);
            driveTrain.driveToDistance(-depotToCrater/3, 2.5);
            driveTrain.turnToDegree(-30,.1,.75);
            driveTrain.driveToDistance((-2*depotToCrater)/3,2.5);
            elevator.lowerMarkerServo();
            sleep(500);
            elevator.raiseMarkerServo();
            driveTrain.driveToDistance(depotToCrater, 5);


        }
        else if(direction == PhoneVision.Direction.RIGHT){
            driveTrain.turnToDegree(-turnAngle, 0.1, 3.0);
            driveTrain.stop();
            driveTrain.driveToDistance(longDistance, 3.5);
            //driveTrain.turnToDegree(2*turnAngle, 0.1, 2.0);
            //driveTrain.driveToDistance(longDistance/4, 3.5);
        }
        else{
            driveTrain.turnToDegree(7.5,.5,1);
            driveTrain.driveToDistance(shortDistance, 3.0);
            //driveTrain.driveToDistance(shortDistance/4, 3.0);
            //check
        }
        driveTrain.stop();
    }
}
