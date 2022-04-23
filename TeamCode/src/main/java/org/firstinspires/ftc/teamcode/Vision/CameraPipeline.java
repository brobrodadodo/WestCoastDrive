package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

import me.wobblyyyy.pathfinder2.time.Time;

public class CameraPipeline extends OpenCvPipeline {
//    private final Scalar LOWER_BOUND = new Scalar(20, 100, 100);
//    private final Scalar UPPER_BOUND = new Scalar(30, 255, 255);
    private final Scalar LOWER_BOUND = new Scalar(36, 100, 150);
    private final Scalar UPPER_BOUND = new Scalar(70, 255, 255);

    private Mat yellowOnly = new Mat();
    private Mat blurred = new Mat();
    private Mat hsvFrame = new Mat();

    private Point cameraCenter = new Point(0, 0);
    private Point ballCenter = new Point(0, 0);
    private Point lastBallCenter = new Point(0, 0);
    private double avgBrightness = 0;

    private double lastDetectionMs;

    public CameraPipeline() {

    }

    @Override
    public void init(Mat mat) {
        super.init(mat);
        int centerX = mat.width() / 2;
        int centerY = mat.height() / 2;
        cameraCenter = new Point(centerX, centerY);
    }

    private Mat filterOnlyYellow(Mat frame) {
        Mat yellowOnly = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvFrame, LOWER_BOUND, UPPER_BOUND, yellowOnly);
        return yellowOnly;
    }

    private Mat blur(Mat frame) {
        Imgproc.GaussianBlur(frame, frame, new Size(5, 5), 0);

        return frame;
    }

    @Override
    public Mat processFrame(Mat input) {
        yellowOnly = filterOnlyYellow(input);
        blurred = blur(yellowOnly);
        Moments moments = Imgproc.moments(blurred, true);
        Point point = new Point(
                moments.m10 / moments.m00,
                moments.m01 / moments.m00
        );
        ballCenter = point;
        Imgproc.cvtColor(blurred, blurred, Imgproc.COLOR_GRAY2RGB);
        Imgproc.circle(blurred, point, 5, new Scalar(20, 255, 255), -1);
        avgBrightness = Core.mean(blurred).val[1];
        return blurred;
    }

    public Point getCameraCenter() {
        return cameraCenter;
    }

    public boolean doesDetectBall() {
        boolean validX = !Double.isNaN(ballCenter.x);
        boolean validY = !Double.isNaN(ballCenter.y);

        return validX && validY;
    }

    public boolean shouldReturnToCenter() {
        boolean detectsBall = doesDetectBall();

        double currentMs = Time.ms();
        if (detectsBall) {
            lastDetectionMs = currentMs;
            return false;
        } else {
            double elapsedMs = currentMs - lastDetectionMs;
            return elapsedMs > 3_000;
        }
    }

    public Point getBallCenter() {
        double x = ballCenter.x;
        double y = ballCenter.y;

        if (Double.isNaN(x) || Double.isNaN(y)) {
            return lastBallCenter;
        }

        lastBallCenter = ballCenter;

        return ballCenter;
    }

    public double getAvgBrightness() {
        return avgBrightness;
    }
}
