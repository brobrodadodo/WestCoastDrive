package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class MarkerDetectorPipeline extends OpenCvPipeline {

    public enum MarkerPosition
    {
        LEFT,
        CENTER,
        RIGHT,
        UPPER,
        LOWER,
        UPPERLEFT,
        LOWERLEFT,
        UPPERRIGHT,
        LOWERRIGHT

    }

    public MarkerDetectorPipeline() {
        this.LOW_GREEN = new Scalar(0, 100, 100);
        this.UPPER_GREEN = new Scalar(30, 255, 255);
    }

    static final Scalar GREEN = new Scalar(0, 255, 0);

    private volatile MarkerPosition position = MarkerPosition.CENTER;

    private Mat region1, region2, region3, region4, region5, region6, region7, region8, region9;

    private Mat greenOnly = new Mat();
    private Mat hsvFrame = new Mat();

    final Scalar LOW_GREEN;
    final Scalar UPPER_GREEN;

    void toGreenOnly(Mat frame) {
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvFrame, LOW_GREEN, UPPER_GREEN, greenOnly);
    }

    void getNineRegions(Mat frame) {
        int firstThird = 220;
        int secondThird = 420;
        int thirdThird = 640;
        int height1 = 120;
        int height2 = 240;oyh
        int height3 = 360;jukl'i'
        region1 = greenOnly.submat(new Rect(0, 0, firstThird, height1));
        region2 = greenOnly.submat(new Rect(firstThird, 0, firstThird, height1));
        region3 = greenOnly.submat(new Rect(secondThird, 0, firstThird, height1));
        region4 = greenOnly.submat(new Rect(0, height1, firstThird, height2));
        region5 = greenOnly.submat(new Rect(firstThird, height1, firstThird, height2));
        region6 = greenOnly.submat(new Rect(secondThird, height1, firstThird, height2));
        region7 = greenOnly.submat(new Rect(0, height2, firstThird, height3));
        region8 = greenOnly.submat(new Rect(firstThird, height2, firstThird, height3));
        region9 = greenOnly.submat(new Rect(secondThird, height2, firstThird, height3));
    }

    @Override
    public void init(Mat mat) {
        super.init(mat);

        toGreenOnly(mat);
        getNineRegions(mat);
    }

    @Override
    public Mat processFrame(Mat input) {

        // Get a green-only image
        toGreenOnly(input);
        // Split it into nine regions
        getNineRegions(input);

        // Get the average amount of green in each region
        double avgLowerLeft = Core.mean(region1).val[0];
        double avgLower = Core.mean(region2).val[0];
        double avgLowerRight = Core.mean(region3).val[0];
        double avgLeft = Core.mean(region4).val[0];
        double avgCenter = Core.mean(region5).val[0];
        double avgRight = Core.mean(region6).val[0];
        double avgUpperLeft = Core.mean(region7).val[0];
        double avgUpper = Core.mean(region8).val[0];
        double avgUpperRight = Core.mean(region9).val[0];

        // Get the max average
        double max1 = Math.max(avgLowerLeft, avgLower);
        double max2 = Math.max(avgLowerRight, avgLeft);
        double max3 = Math.max(avgCenter, avgRight);
        double max4 = Math.max(avgUpperLeft,avgUpper);
        double max5 = Math.max(max1, max2);
        double max6 = Math.max(max3, max4);
        double max7 = Math.max(max5, max6);
        double max = Math.max(max7, avgUpperRight);

        Point rectA;
        Point rectB;

        // Backtrack and check which was the max, and set the position to it.
        double height = input.height();
        int height1 = (int)height/3;
        int height2 = (int)height*2/3;
        int height3 = (int)height;

        if (max == avgLeft) {
            position = MarkerPosition.LEFT;
            rectA = new Point(0, height1);
            rectB = new Point(firstThird, height2);
        } else if (max == avgCenter) {
            position = MarkerPosition.CENTER;
            rectA = new Point(firstThird, height1);
            rectB = new Point(secondThird, height2);
        } else if (max == avgRight) {
            position = MarkerPosition.RIGHT;
            rectA = new Point(secondThird, height1);
            rectB = new Point(input.width(), height2);
        } else if (max == avgLowerLeft) {
            position = MarkerPosition.LOWERLEFT;
            rectA = new Point(0, 0);
            rectB = new Point(firstThird, height1);
        } else if (max == avgLower) {
            position = MarkerPosition.LOWER;
            rectA = new Point(firstThird, 0);
            rectB = new Point(secondThird, height1);
        } else if (max == avgLowerRight) {
            position = MarkerPosition.LOWERRIGHT;
            rectA = new Point(secondThird, 0);
            rectB = new Point(input.width(), height1);
        } else if(max == avgUpperLeft) {
            position = MarkerPosition.UPPERLEFT;
            rectA = new Point(0, height2);
            rectB = new Point(firstThird, height3);
        } else if (max == avgUpper) {
            position = MarkerPosition.UPPER;
            rectA = new Point(firstThird, height2);
            rectB = new Point(secondThird, height3);
        } else {
            position = MarkerPosition.UPPERRIGHT;
            rectA = new Point(secondThird, height2);
            rectB = new Point(input.width(), height3);
        }

        // Draw a rectangle on the frame around the selected third
        Imgproc.rectangle(input, rectA, rectB, GREEN, 5);

        return input;
    }

    public MarkerPosition getPosition() {
        return position;
    }
}