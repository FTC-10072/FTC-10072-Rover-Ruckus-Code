package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotClasses.Elevator;
import org.firstinspires.ftc.teamcode.RobotClasses.HardwareRobot;
import org.firstinspires.ftc.teamcode.RobotClasses.PhoneVision;

@Autonomous(name="DepotAutoWithDisengage", group="Autonomous")
public class DepotAutoWithDisengage extends LinearOpMode {
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
        elevator.detachFromLander();
        driveTrain.turnToDegree(-15,.5, .5);
        elevator.moveElevatorDown();
        driveTrain.turnToDegree(15,.5, .5);


        double turnAngle = 27.565;
        double shortDistance = Math.sqrt(8.0) * 12 - 2;
        double longDistance = Math.sqrt(10.0) * 12 - 3;
        int depotToCrater = 72;
        //scans for location of mineral
        driveTrain.driveToDistance(4, .75);
        driveTrain.turnToDegree(85, .1, 1.5);
        driveTrain.stop();
        driveTrain.driveToDistance(-3.5,.75);
        PhoneVision.Direction direction = vision.getDirection();
        //driveTrain.driveToDistance(3.5,.75);
        driveTrain.turnToDegree(-75, .1, 1.5);
        driveTrain.stop();
        //moves to the location that is provided by the vision
        if(direction == PhoneVision.Direction.LEFT){
            /*driveTrain.turnToDegree(turnAngle, 0.1, 1.0);
            driveTrain.stop();
            driveTrain.driveToDistance(longDistance+2.5, 2.5);
            //driveTrain.turnToDegree(-2*turnAngle, 0.1, 3.0);
            //driveTrain.driveToDistance(longDistance*2/5, 3.5);
            driveTrain.turnToDegree(-(turnAngle+60), 0.1,1.5);
            driveTrain.stop();
            driveTrain.driveToDistance(15, 1.0);
            driveTrain.turnToDegree(180, .1, 2.0);
            driveTrain.stop();
            elevator.lowerMarkerServo();
            sleep(500);
            driveTrain.turnToDegree(180, .1, 2.0);
            driveTrain.stop();
            //driveTrain.turnToDegree(-45, .5, 2.5);
            elevator.raiseMarkerServo();
            driveTrain.driveToDistance(-72, 1.0);*/
            driveTrain.turnToDegree(turnAngle, 0.1, 1.0);
            driveTrain.stop();
            driveTrain.driveToDistance(longDistance+4.5, 2.5);
            //driveTrain.turnToDegree(2*turnAngle, 0.1, 3.0);
            //driveTrain.driveToDistance(longDistance*2/5, 3.5);
            driveTrain.turnToDegree(-turnAngle-50, 0.1,2.0);
            driveTrain.stop();
            driveTrain.driveToDistance(20, 1.0);
            driveTrain.turnToDegree(-220, .1, 1.5);
            driveTrain.driveToDistance(-3,.5);
            driveTrain.stop();
            elevator.lowerMarkerServo();
            sleep(1500);

            driveTrain.stop();
            //driveTrain.turnToDegree(45, .5, 2.5);
            elevator.raiseMarkerServo();
            driveTrain.driveToDistance(72, 5.0);

        }
        else if(direction == PhoneVision.Direction.RIGHT){
            driveTrain.turnToDegree(-turnAngle, 0.1, 1.0);
            driveTrain.stop();
            driveTrain.driveToDistance(longDistance+2.5, 2.5);
            //driveTrain.turnToDegree(2*turnAngle, 0.1, 3.0);
            //driveTrain.driveToDistance(longDistance*2/5, 3.5);
            driveTrain.turnToDegree(turnAngle+45, 0.1,2.0);
            driveTrain.stop();
            driveTrain.driveToDistance(20, 1.0);
            driveTrain.turnToDegree(90, .1, 1.5);
            driveTrain.stop();
            elevator.lowerMarkerServo();
            sleep(1500);
            driveTrain.turnToDegree(90, .1, 1.5);
            driveTrain.stop();
            //driveTrain.turnToDegree(45, .5, 2.5);
            elevator.raiseMarkerServo();
            driveTrain.driveToDistance(72, 5.0);

        }
        else{
            driveTrain.driveToDistance(shortDistance+50, 2.5);
            driveTrain.turnToDegree(120, .1, 1.5);
            driveTrain.stop();
            elevator.lowerMarkerServo();
            sleep(1500);
            /*driveTrain.turnToDegree(180, .1, 1.5);
            driveTrain.stop();
            driveTrain.turnToDegree(-90, .1, 2.5);
            driveTrain.stop();
            elevator.raiseMarkerServo();
            driveTrain.driveToDistance(-20,2.0);
            driveTrain.turnToDegree(40, 0.1, 1.5);
            driveTrain.stop();
            driveTrain.driveToDistance(-72,5.0);*/
            //driveTrain.turnToDegree(-90,.5, .75);
            driveTrain.turnToDegree(-35,.1,.5);
            driveTrain.driveToDistance(72, 5.0);

        }


        driveTrain.stop();
        driveTrain.driveToDistance(-6,3);
        driveTrain.stop();
    }
}