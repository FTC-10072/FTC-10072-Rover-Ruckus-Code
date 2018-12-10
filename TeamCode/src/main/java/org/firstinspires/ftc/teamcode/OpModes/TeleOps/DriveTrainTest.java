package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotClasses.HardwareRobot;

@TeleOp(name = "DriveTrain Test", group = "Testing")
//@Disabled
public class DriveTrainTest extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot();
    DriveTrain driveTrain = new DriveTrain();

    double currentP = 0.072;
    double currentI = 0.0;
    double currentD = 0.00420;
    double change =   0.0001;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        driveTrain.init(robot, this);
        waitForStart();

        double precision = 0.05;
        while (opModeIsActive()){
            if(gamepad1.dpad_left){
                if(gamepad1.b){
                    driveTrain.turnToDegree(45.0, precision, 5.0);
                }else if(gamepad1.a){
                    driveTrain.turnToDegree(90.0, precision, 10.0);
                }else if(gamepad1.x){
                    driveTrain.turnToDegree(135.0, precision, 15.0);
                }
                else if (gamepad1.y){
                    driveTrain.turnToDegree(180.0, precision, 20.0);
                }
            }
            else if(gamepad1.dpad_right){
                if(gamepad1.b){
                    driveTrain.turnToDegree(-45.0, precision, 5.0);
                }else if(gamepad1.a){
                    driveTrain.turnToDegree(-90.0, precision, 10.0);
                }else if(gamepad1.x){
                    driveTrain.turnToDegree(-135.0, precision, 15.0);
                }
                else if (gamepad1.y){
                    driveTrain.turnToDegree(-180.0, precision, 20.0);
                }
            }
            else if(gamepad1.dpad_up){
                if(gamepad1.b){
                    driveTrain.driveToDistance(12.0, 5);
                }else if(gamepad1.a){
                    driveTrain.driveToDistance(24.0, 10);
                }else if(gamepad1.x){
                    driveTrain.driveToDistance(36.0, 15);
                }
                else if (gamepad1.y){
                    driveTrain.driveToDistance(48.0, 20);
                }
            }
            else if(gamepad1.dpad_down){
                if(gamepad1.b){
                    driveTrain.driveToDistance(-12.0, 5);
                }else if(gamepad1.a){
                    driveTrain.driveToDistance(-24.0, 10);
                }else if(gamepad1.x){
                    driveTrain.driveToDistance(-36.0, 15);
                }
                else if (gamepad1.y){
                    driveTrain.driveToDistance(-48.0, 20);
                }
            }
            else if(gamepad1.left_bumper){
                if(gamepad1.x){
                    currentP -= change;
                }else if(gamepad1.a){
                    currentI -= change;
                }else if(gamepad1.b){
                    currentD -= change;
                }
                driveTrain.changePID(currentP, currentI, currentD);
                sleep(200);
            }
            else if(gamepad1.right_bumper){
                if(gamepad1.x){
                    currentP += change;
                }else if(gamepad1.a){
                    currentI += change;
                }else if(gamepad1.b){
                    currentD += change;
                }
                driveTrain.changePID(currentP, currentI, currentD);
                sleep(200);
            }
        }
        driveTrain.stop();
    }
}
