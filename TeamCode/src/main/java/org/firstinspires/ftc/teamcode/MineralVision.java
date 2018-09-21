package org.firstinspires.ftc.teamcode;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class MineralVision extends OpenCVPipeline{

    private static final double MIN_MASS = 6.0;

    private Mat hsv = new Mat();

    private MineralResult result;

    public synchronized MineralResult getResult() {
        return result;
    }

    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        // convert to HSV
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV);

        // h range is 0 - 179
        // s range is 0 - 255
        // v range is 0 - 255

        List<Scalar> hsvMin = new ArrayList<>();
        List<Scalar> hsvMax = new ArrayList<>();

        // gold
        hsvMin.add(new Scalar(10/2, 133, 186));
        hsvMax.add(new Scalar(50/2, 255, 255));
        // silver
        hsvMin.add(new Scalar(45/2, 0, 209));
        hsvMax.add(new Scalar(112/2, 64, 255));

        // make a list of channels that are blank (used for combining binary images)
        List<Mat> channels = new ArrayList<>();
        double[] maxMass = {Double.MIN_VALUE, Double.MIN_VALUE, Double.MIN_VALUE};
        int[] maxMassIndex = {2, 2, 2};

        // We are about to loop over the filters and compute the "color mass" for each
        // color on each side of the image.
        // These variables are used inside the loop:
        Mat maskedImage = new Mat();
        Mat colSum = new Mat();
        double mass;
        int[] data = new int[2];
        int step = hsv.width()/3;

        // loop through colors
        for(int i=0; i< 2;i++){
            ImageUtil.hsvInRange(hsv, hsvMin.get(i), hsvMax.get(i), maskedImage);

            Core.reduce(maskedImage, colSum, 0, Core.REDUCE_SUM, 4);
            channels.add(maskedImage.clone());
            int start = 0;
            int end = step;
            for(int j=0; j<3;j++){
                // get mass in third
                mass = 0;
                for(int x = start;x<end;x++){
                    colSum.get(0, x, data);
                    mass += data[0];
                }
                mass /= hsv.size().area();
                // check if mass is greater
                if(mass > MIN_MASS && mass > maxMass[j]){
                    maxMass[j] = mass;
                    maxMassIndex[j] = i;
                }
                // iter
                start += step;
                end += step;
            }
        }
        // merge channels
        Core.merge(channels, rgba);

        // get max masses
        MineralResult.MineralColor[] mineralColors = MineralResult.MineralColor.values();
        MineralResult.MineralColor left = mineralColors[maxMassIndex[0]];
        MineralResult.MineralColor middle = mineralColors[maxMassIndex[1]];
        MineralResult.MineralColor right = mineralColors[maxMassIndex[2]];

        // draw color result bars
        int barHeight = hsv.height() / 30;
        Imgproc.rectangle(rgba, new Point(0,0), new Point(step, barHeight), left.color, barHeight);
        Imgproc.rectangle(rgba, new Point(step, 0), new Point(step*2, barHeight), middle.color, barHeight);
        Imgproc.rectangle(rgba, new Point(step*2, 0), new Point(hsv.width(), barHeight), right.color, barHeight);

        result = new MineralResult(left, middle, right);
        return rgba;
    }
}
