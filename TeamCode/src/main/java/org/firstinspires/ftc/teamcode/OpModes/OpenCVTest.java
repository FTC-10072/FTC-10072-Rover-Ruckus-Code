package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.MineralVision;
import org.opencv.android.CameraBridgeViewBase;

@Autonomous(name = "OpenCV Test", group = "Testing")
//@Disabled
public class OpenCVTest extends LinearOpMode {
    private MineralVision mineralVision;

    @Override
    public void runOpMode() throws InterruptedException {
        mineralVision = new MineralVision();
        mineralVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        mineralVision.enable();
        while(opModeIsActive()){

        }
        mineralVision.disable();
    }
}
