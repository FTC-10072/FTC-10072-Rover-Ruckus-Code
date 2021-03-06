package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotClasses.Elevator;
import org.firstinspires.ftc.teamcode.RobotClasses.HardwareRobot;
import org.firstinspires.ftc.teamcode.RobotClasses.PhoneVision;

@Autonomous(name="CraterFull", group="Autonomous")
public class CraterFull extends LinearOpMode {
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
        double longDistance = Math.sqrt(10.0) * 12 - 8 /*originally - 3*/;
        double halftileDistance = Math.sqrt(324);
        double depotToCraterDistance = 72;
        driveTrain.driveToDistance(4, .75);
        driveTrain.turnToDegree(90, .1, 2.5);
        driveTrain.driveToDistance(-3.5,.75);
        PhoneVision.Direction direction = vision.getDirection();
        driveTrain.driveToDistance(3.5,.75);
        driveTrain.turnToDegree(-90, .1, 2.5);
        if(direction == PhoneVision.Direction.LEFT){

            driveTrain.turnToDegree(turnAngle, 0.1, 2.0);
            driveTrain.driveToDistance(longDistance, 3.5);
            driveTrain.turnToDegree(135,.1,2.5);
            driveTrain.driveToDistance(depotToCraterDistance,5.0);
            elevator.lowerMarkerServo();
            driveTrain.driveToDistance(-depotToCraterDistance, 5.0);

            //driveTrain.turnToDegree(-2*turnAngle, 0.1, 2.0);
            //driveTrain.driveToDistance(longDistance/4, 3.5);



        }
        else if(direction == PhoneVision.Direction.RIGHT){
            driveTrain.turnToDegree(-turnAngle, 0.1, 2.0);
            driveTrain.driveToDistance(longDistance, 3.5);
            driveTrain.driveToDistance(-5,2.0);
            driveTrain.turnToDegree(95,.1,1.5);
            driveTrain.driveToDistance(3.5*halftileDistance,2.5);
            driveTrain.turnToDegree(45,.1 , 1.5);
            driveTrain.driveToDistance(depotToCraterDistance-18, 4.0);
            elevator.lowerMarkerServo();
            driveTrain.driveToDistance(-depotToCraterDistance, 5.0);

            //driveTrain.turnToDegree(2*turnAngle, 0.1, 2.0);
            //driveTrain.driveToDistance(longDistance/4, 3.5);
        }
        else{
            driveTrain.driveToDistance(shortDistance, 3.0);
            //driveTrain.driveToDistance(shortDistance/4, 3.0);
        }
        driveTrain.stop();
    }
}
