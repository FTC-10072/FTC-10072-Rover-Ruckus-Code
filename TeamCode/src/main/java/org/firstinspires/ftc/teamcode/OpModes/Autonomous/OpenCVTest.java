package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.MineralResult;
import org.firstinspires.ftc.teamcode.MineralVision;
import org.firstinspires.ftc.teamcode.RobotClasses.PhoneVision;
import org.opencv.android.CameraBridgeViewBase;

import org.firstinspires.ftc.teamcode.RobotClasses.PhoneVision.Direction;

@Autonomous(name = "OpenCV Test", group = "Testing")
//@Disabled
public class OpenCVTest extends LinearOpMode {
    private PhoneVision vision;

    @Override
    public void runOpMode() throws InterruptedException {
        vision = new PhoneVision(this);
        waitForStart();
        Direction dir = vision.getDirection();
        switch (dir){
            case LEFT:
                telemetry.addData("Direction", "left");
                break;
            case MIDDLE:
                telemetry.addData("Direction", "middle");
                break;
            case RIGHT:
                telemetry.addData("Direction", "right");
                break;
            default:
                telemetry.addData("Direction", "unknown");
        }
        telemetry.update();
        while(opModeIsActive()){}
    }
}
