package org.firstinspires.ftc.teamcode;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

class contourComparator implements Comparator<MatOfPoint> {
    @Override
    public int compare(MatOfPoint o1, MatOfPoint o2) {
        return (int)(10*Imgproc.contourArea(o2) - 10*Imgproc.contourArea(o1));
    }
}

public class MineralVision extends OpenCVPipeline{

    private LinearOpMode opMode;
    private Mat hsv = new Mat();

    private MineralResult result = new MineralResult(MineralResult.MineralColor.UNKNOWN,
            MineralResult.MineralColor.UNKNOWN, MineralResult.MineralColor.UNKNOWN);

    public synchronized MineralResult getResult() {
        return result;
    }

    public MineralVision(LinearOpMode opMode){
        this.opMode = opMode;
    }

    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        // convert to HSV

        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV);

        Mat goldMask = new Mat();
        Mat silverMask = new Mat();

        // h range is 0 - 179
        // s range is 0 - 255
        // v range is 0 - 255
        Core.inRange(hsv, new Scalar(11, 114, 101), new Scalar(42, 255, 255), goldMask);
        Core.inRange(hsv, new Scalar(0, 0, 220), new Scalar(180, 49, 255), silverMask);

        // use masks to filter out all other data
        Mat goldImage = new Mat();
        Mat silverImage = new Mat();
        Mat mineralImage = new Mat();
        rgba.copyTo(goldImage, goldMask);
        rgba.copyTo(silverImage, silverMask);
        Core.add(goldImage, silverImage, mineralImage);
        // release mats
        goldImage.release();
        silverImage.release();

        // get contours
        List<MatOfPoint> goldContours = new ArrayList<>();
        List<MatOfPoint> silverContours = new ArrayList<>();
        Imgproc.findContours(goldMask, goldContours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(silverMask, silverContours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        // release mats
        goldMask.release();
        silverMask.release();

        // find largest goldContour and 2 largest silverContours
        MatOfPoint goldContour = null, silverContour1 = null, silverContour2 = null;
        Collections.sort(goldContours, new contourComparator());
        Collections.sort(silverContours, new contourComparator());
        if(goldContours.size() > 0) goldContour = goldContours.get(0);
        if(silverContours.size() > 0) silverContour1 = silverContours.get(0);
        if(silverContours.size() > 1) silverContour2 = silverContours.get(1);

        // get bounding boxes and draw them
        Rect goldRect = null, silverRect1 = null, silverRect2 = null;
        if(goldContour != null) goldRect = Imgproc.boundingRect(goldContour);
        if(silverContour1 != null) silverRect1 = Imgproc.boundingRect(silverContour1);
        if(silverContour2 != null) silverRect2 = Imgproc.boundingRect(silverContour2);

        if(goldRect != null) Imgproc.rectangle(mineralImage,
                goldRect.tl(), goldRect.br(), ImageUtil.YELLOW);
        if(silverRect1 != null) Imgproc.rectangle(mineralImage,
                silverRect1.tl(), silverRect1.br(), ImageUtil.WHITE);
        if(silverRect2 != null) Imgproc.rectangle(mineralImage,
                silverRect2.tl(), silverRect2.br(), ImageUtil.WHITE);

        // get position
        int goldPos = -1;
        int silverPos1 = -1;
        int silverPos2 = -1;

        // we look at y values because the phone is on its side
        if(goldRect != null){
            goldPos = goldRect.y + goldRect.height / 2;
        }
        if(silverRect1 != null){
            silverPos1 = silverRect1.y + silverRect1.height / 2;
        }
        if(silverRect2 != null){
            silverPos2 = silverRect2.y + silverRect2.height / 2;
        }

        // find which third of image each mineral is in
        int totalHeight = rgba.height();
        int thirdHeight = totalHeight / 3;

        Imgproc.line(mineralImage, new Point(0, thirdHeight), new Point(rgba.width(), thirdHeight), ImageUtil.RED, 2);
        Imgproc.line(mineralImage, new Point(0, 2*thirdHeight), new Point(rgba.width(),2*thirdHeight), ImageUtil.RED, 2);
        Imgproc.rectangle(mineralImage, new Point(0, 0), new Point(rgba.width(), rgba.height()), ImageUtil.RED, 2);

        int goldThird = 3;
        int silverThird1 = 3;
        int silverThird2 = 3;
        if(goldPos != -1) goldThird = goldPos / thirdHeight;
        if(silverPos1 != -1) silverThird1 = silverPos1 / thirdHeight;
        if(silverPos2 != -1) silverThird2 = silverPos2 / thirdHeight;

        // set colors for result
        MineralResult.MineralColor colors[] = {MineralResult.MineralColor.UNKNOWN,
                MineralResult.MineralColor.UNKNOWN,
                MineralResult.MineralColor.UNKNOWN,
                MineralResult.MineralColor.UNKNOWN};
        colors[silverThird1] = MineralResult.MineralColor.WHITE;
        colors[silverThird2] = MineralResult.MineralColor.WHITE;
        colors[goldThird] = MineralResult.MineralColor.YELLOW;

        // set result
        result = new MineralResult(colors[0], colors[1], colors[2]);

        return mineralImage;
    }
}
