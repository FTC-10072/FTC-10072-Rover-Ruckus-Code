package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.MineralResult;
import org.firstinspires.ftc.teamcode.MineralVision;

import static org.firstinspires.ftc.teamcode.MineralResult.MineralColor.YELLOW;
import static org.firstinspires.ftc.teamcode.MineralResult.MineralColor.WHITE;

public class PhoneVision {
    private MineralVision vision;
    private LinearOpMode opMode;

    public enum Direction{
        LEFT, MIDDLE, RIGHT, UNKNOWN;
    }

    public PhoneVision(LinearOpMode opMode){
        vision = new MineralVision(opMode);
        vision.init(opMode.hardwareMap.appContext, CameraViewDisplay.getInstance());
        this.opMode = opMode;
    }

    public Direction getDirection(){
        vision.enable();
        opMode.sleep(2000);
        MineralResult mineralResult = vision.getResult();
        vision.disable();
        MineralResult.MineralColor left = mineralResult.getLeftColor();
        MineralResult.MineralColor middle = mineralResult.getMidColor();
        MineralResult.MineralColor right = mineralResult.getRightColor();
        // if gold is detected, go that way
        // or if all other ways are white, go that way
        // otherwise return unknown
        if((middle == WHITE && right == WHITE) || left == YELLOW){
            return Direction.LEFT;
        }
        else if((left == WHITE && right == WHITE) || middle == YELLOW){
            return Direction.MIDDLE;
        }
        else if((left == WHITE && middle == WHITE) || right == YELLOW){
            return Direction.RIGHT;
        }
        else{
            return Direction.UNKNOWN;
        }
    }
}
