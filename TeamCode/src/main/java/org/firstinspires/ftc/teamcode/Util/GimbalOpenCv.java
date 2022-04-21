package org.firstinspires.ftc.teamcode.Util;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;



public class GimbalOpenCv extends OpenCvPipeline {

    public enum MarkerPosition
    {
        LEFT,
        CENTER,
        RIGHT
    }

    final boolean blue;

    public GimbalOpenCv(boolean blue) {
        this.blue = blue;
        this.LOW_GREEN = new Scalar(blue ? 100 : 0, 100, 100);
        this.UPPER_GREEN = new Scalar(blue ? 150 : 30, 255, 255);
    }

    static final Scalar GREEN = new Scalar(0, 255, 0);

    private volatile MarkerPosition position = MarkerPosition.LEFT;

    private Mat region1, region2, region3;

    private Mat greenOnly = new Mat();
    private Mat hsvFrame = new Mat();

    final Scalar LOW_GREEN;
    final Scalar UPPER_GREEN;

    private int firstThird, secondThird;

    void toGreenOnly(Mat frame) {
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvFrame, LOW_GREEN, UPPER_GREEN, greenOnly);
    }

    void getThreeRegions(Mat frame) {
        int height = frame.height();
        region1 = greenOnly.submat(new Rect(0, 0, firstThird, height));
        region2 = greenOnly.submat(new Rect(firstThird, 0, firstThird, height));
        region3 = greenOnly.submat(new Rect(secondThird, 0, firstThird, height));
    }

    @Override
    public void init(Mat mat) {
        super.init(mat);
        firstThird = mat.width()/3;
        secondThird = mat.width()/3 * 2;

        toGreenOnly(mat);
        getThreeRegions(mat);
    }

    @Override
    public Mat processFrame(Mat input) {

        // Get a green-only image
        toGreenOnly(input);
        // Split it into three regions
        getThreeRegions(input);

        // Get the average amount of green in each region
        double avgLeft = Core.mean(region1).val[0];
        double avgMid = Core.mean(region2).val[0];
        double avgRight = Core.mean(region3).val[0];

        // Get the max average
        double max = Math.max(Math.max(avgLeft, avgMid), avgRight);

        Point rectA;
        Point rectB;

        // Backtrack and check which was the max, and set the position to it.
        if (max == avgLeft) {
            position = MarkerPosition.LEFT;
            rectA = new Point(0, input.height()/2);
            rectB = new Point(firstThird, input.height());
        } else if (max == avgMid) {
            position = MarkerPosition.CENTER;
            rectA = new Point(firstThird, input.height()/2);
            rectB = new Point(secondThird, input.height());
        } else {
            position = MarkerPosition.RIGHT;
            rectA = new Point(secondThird, input.height()/2);
            rectB = new Point(input.width(), input.height());
        }

        // Draw a rectangle on the frame around the selected third
        Imgproc.rectangle(input, rectA, rectB, GREEN, 5);

        return input;
    }

    public MarkerPosition getPosition() {
        return position;
    }
}
