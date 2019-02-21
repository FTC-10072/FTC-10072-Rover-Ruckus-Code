package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.Arm;
import org.firstinspires.ftc.teamcode.RobotClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotClasses.Elevator;
import org.firstinspires.ftc.teamcode.RobotClasses.HardwareRobot;

@Autonomous (name = "DisengageTest", group = "Testing")
//@Disabled
public class DisengageTest extends LinearOpMode {
    HardwareRobot robot = new HardwareRobot();
    DriveTrain driveTrain = new DriveTrain();
    Elevator elevator = new Elevator();
    Arm arm = new Arm();
    //PhoneVision vision = new PhoneVision(this);

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        driveTrain.init(robot, this);
        elevator.init(robot, this);
        double shortDistance = Math.sqrt(8.0) * 12 - 2;


        elevator.detachFromLander();
        driveTrain.turnToDegree(-15,.5, 1.0);
        driveTrain.stop();
        elevator.moveElevatorDown();
        driveTrain.turnToDegree(15,.5, 1.0);
        driveTrain.stop();
        driveTrain.driveToDistance(shortDistance+15, 3.0);

        elevator.lowerMarkerServo();
        driveTrain.turnToDegree(90, .1, 2.5);
        driveTrain.stop();
        elevator.raiseMarkerServo();
        // ensure ratchet is locked
       /* elevator.setRatchetLock(true);
        elevator.closeClaw();
        elevator.moveElevatorDown();
        waitForStart();

        //move up a bit so the ratchet can disengage
        elevator.moveElevator(-0.5);
        elevator.setRatchetLock(false);
        sleep(1000);
        elevator.moveElevator(0.0);
        elevator.moveElevatorToHookHeight();
        sleep(4000);
        elevator.openClaw();
        sleep(1000);
        elevator.moveElevatorDown();
        sleep(4000);*/
    }
}