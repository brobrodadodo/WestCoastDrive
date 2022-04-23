package org.firstinspires.ftc.teamcode.WestCoastDrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.CameraPipeline;
import org.firstinspires.ftc.teamcode.Vision.GimbalMotion;
import org.firstinspires.ftc.teamcode.Vision.MarkerDetectorPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import me.wobblyyyy.pathfinder2.control.ProportionalController;
import me.wobblyyyy.pathfinder2.math.MinMax;
import me.wobblyyyy.pathfinder2.time.ElapsedTimer;

@TeleOp(name = "VisionTest", group = "default")
public class VisionTest extends LinearOpMode {
    private CameraPipeline pipeline;

    private ProportionalController yawController = new ProportionalController(-0.000010);
    private ProportionalController pitchController = new ProportionalController(-0.000010);

    private Servo yaw;
    private Servo pitch;
    private GimbalMotion gimbal;

    private void adjustGimbal(Point cameraCenter, Point ballCenter) {
        if (pipeline.shouldReturnToCenter()) {
            gimbal.resetPosition();
        }

        double deltaYaw = MinMax.clip(ballCenter.x - cameraCenter.x, -320, 320);
        double deltaPitch = MinMax.clip(ballCenter.y - cameraCenter.y, -320, 320);

        double yawAdjustment = MinMax.clip(yawController.calculate(deltaYaw, 0.0), -0.005, 0.005);
        double pitchAdjustment = MinMax.clip(pitchController.calculate(deltaPitch, 0.0), -0.0025, 0.0025);

        double currentYaw = yaw.getPosition();
        double currentPitch = pitch.getPosition();

        double nextYaw = MinMax.clip(currentYaw + yawAdjustment, 0, 1);
        double nextPitch = MinMax.clip(currentPitch + pitchAdjustment, 0, 0.6);

        yaw.setPosition(nextYaw);
        pitch.setPosition(nextPitch);
    }

    @Override
    public void runOpMode() {
        // yaw - rotation (x/y)
        // pitch - tilt (up/down)
        yaw = hardwareMap.get(Servo.class, "webcamYaw");
        pitch = hardwareMap.get(Servo.class, "webcamPitch");

        gimbal = new GimbalMotion(yaw, pitch);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName()
        );
        gimbal.resetPosition();
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(
                webcamName,
                cameraMonitorViewId
        );
        pipeline = new CameraPipeline();
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        Point cameraCenter = pipeline.getCameraCenter();
        Point ballCenter = pipeline.getBallCenter();

        adjustGimbal(cameraCenter, ballCenter);

        while (opModeIsActive()) {
            adjustGimbal(pipeline.getCameraCenter(), pipeline.getBallCenter());

            telemetry.addData("cameraCenter", cameraCenter);
            telemetry.addData("ballCenter", ballCenter);
            telemetry.addData("avgBrightness", pipeline.getAvgBrightness());
            telemetry.update();

//            sleep(1_000);
        }
    }
}
