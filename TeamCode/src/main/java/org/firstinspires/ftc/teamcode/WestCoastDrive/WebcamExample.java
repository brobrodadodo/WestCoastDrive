
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class WebcamExample extends LinearOpMode
{
    OpenCvWebcam webcam;
    SamplePipeline pipeline;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Type", pipeline.getType());
            telemetry.addData("Average", pipeline.getAverage());
            telemetry.update();

            sleep(100);
        }
    }

    public static class SamplePipeline extends OpenCvPipeline
    {
        private static final Scalar BLUE = new Scalar(0,0,255);
        Point topLeft = new Point(50,50);
        Point bottomRight = new Point(100,100);
        private static final int THRESHOLD = 107;
        Mat region;
        Mat YCrCb;
        Mat Cb = new Mat();
        private volatile int average;
        private volatile TYPE type;

        private void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat mat) {
            inputToCb(mat);
            region = Cb.submat(new Rect(topLeft, bottomRight));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);
            average = (int) Core.mean(region).val[0];
            Imgproc.rectangle(input, topLeft, bottomRight, BLUE, 2);

            if (average > THRESHOLD) {
                type = TYPE.BALL;
            }
            else {
                type = TYPE.NONE;
            }
            return input;
        }

        public TYPE getType(){
            return type;
        }

        public int getAverage() {
            return average;
        }

        enum TYPE {
            BALL, NONE
        }
    }
}