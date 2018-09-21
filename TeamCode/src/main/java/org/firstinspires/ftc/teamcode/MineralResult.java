package org.firstinspires.ftc.teamcode;

import org.opencv.core.Scalar;

public class MineralResult {
    public MineralResult(MineralColor leftColor, MineralColor midColor, MineralColor rightColor) {
        this.leftColor = leftColor;
        this.midColor = midColor;
        this.rightColor = rightColor;
    }

    public MineralColor getLeftColor() {
        return leftColor;
    }

    public MineralColor getMidColor() {
        return midColor;
    }

    public MineralColor getRightColor() {
        return rightColor;
    }

    public enum MineralColor{
        YELLOW (ImageUtil.YELLOW),
        WHITE (ImageUtil.WHITE),
        UNKNOWN (ImageUtil.BLACK);

        public final Scalar color;

        MineralColor(Scalar color) {
            this.color = color;
        }
    }

    private final MineralColor leftColor, midColor, rightColor;

    @Override
    public String toString(){
        return "" + leftColor + " , " + midColor + " , " + rightColor;
    }
}
