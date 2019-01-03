package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotClasses.Elevator;
import org.firstinspires.ftc.teamcode.RobotClasses.HardwareRobot;
import org.firstinspires.ftc.teamcode.RobotClasses.PhoneVision;

@Autonomous(name="DepotAuto", group="Autonomous")
public class DepotAuto extends LinearOpMode {
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
        double longDistance = Math.sqrt(10.0) * 12 -3;
        double depotToCrater = 72;
        //scans for location of mineral
        driveTrain.driveToDistance(4, .75);
        driveTrain.turnToDegree(90, .1, 3.5);
        driveTrain.driveToDistance(-3.5,.75);
        PhoneVision.Direction direction = vision.getDirection();
        driveTrain.driveToDistance(3.5,.75);
        driveTrain.turnToDegree(-90, .1, 3.5);
        //moves to the location that is provided by the vision
        if(direction == PhoneVision.Direction.LEFT){
            driveTrain.turnToDegree(turnAngle, 0.1, 3.0);
            driveTrain.driveToDistance(longDistance+6, 3.5);
            //driveTrain.turnToDegree(-2*turnAngle, 0.1, 3.0);
            //driveTrain.driveToDistance(longDistance*2/5, 3.5);
            driveTrain.turnToDegree(-(turnAngle+40), 0.1,2.0);
            driveTrain.driveToDistance(15, 1.0);
            elevator.lowerMarkerServo();
            sleep(500);
            //driveTrain.turnToDegree(-45, .5, 2.5);
            elevator.raiseMarkerServo();
            driveTrain.driveToDistance(-depotToCrater, 5.0);
        }
        else if(direction == PhoneVision.Direction.RIGHT){
            driveTrain.turnToDegree(-turnAngle, 0.1, 3.0);
            driveTrain.driveToDistance(longDistance+2.5, 3.5);
            //driveTrain.turnToDegree(2*turnAngle, 0.1, 3.0);
            //driveTrain.driveToDistance(longDistance*2/5, 3.5);
            driveTrain.turnToDegree(turnAngle+50, 0.1,2.0);
            driveTrain.driveToDistance(15, 1.0);
            elevator.lowerMarkerServo();
            sleep(500);
            //driveTrain.turnToDegree(45, .5, 2.5);
            elevator.raiseMarkerServo();
            driveTrain.driveToDistance(-depotToCrater, 5.0);

        }
        else{
            driveTrain.driveToDistance(shortDistance+15, 3.0);

            elevator.lowerMarkerServo();
            sleep(500);
            driveTrain.turnToDegree(-90, .1, 2.5);
            elevator.raiseMarkerServo();
            driveTrain.driveToDistance(-17,2.0);
            driveTrain.turnToDegree(40, 0.1, 1.5);
            driveTrain.driveToDistance(-depotToCrater,5.0);
        }


        driveTrain.stop();
        driveTrain.driveToDistance(-6,3);
        driveTrain.stop();
    }
}