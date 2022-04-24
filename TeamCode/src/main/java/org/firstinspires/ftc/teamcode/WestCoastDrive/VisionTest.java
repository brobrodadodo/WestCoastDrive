package org.firstinspires.ftc.teamcode.WestCoastDrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.CameraPipeline;
import org.firstinspires.ftc.teamcode.Vision.GimbalMotion;
import org.firstinspires.ftc.teamcode.Vision.MarkerDetectorPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import me.wobblyyyy.pathfinder2.Pathfinder;
import me.wobblyyyy.pathfinder2.control.ProportionalController;
import me.wobblyyyy.pathfinder2.math.MinMax;
import me.wobblyyyy.pathfinder2.robot.components.MultiMotor;
import me.wobblyyyy.pathfinder2.time.ElapsedTimer;

@TeleOp(name = "VisionTest", group = "default")
public class VisionTest extends LinearOpMode {
    private CameraPipeline pipeline;

    private ProportionalController yawController = new ProportionalController(-0.000015);
    private ProportionalController pitchController = new ProportionalController(-0.000015);

    private Servo yaw;
    private Servo pitch;
    private GimbalMotion gimbal;

    private double yawAdjustment = 0.0;
    private double pitchAdjustment = 0.0;
    private boolean isShooterActive = false;

    private void adjustGimbal(Point cameraCenter, Point ballCenter) {
        if (pipeline.shouldReturnToCenter()) {
            gimbal.resetPosition();
        }

        double deltaYaw = MinMax.clip(ballCenter.x - cameraCenter.x, -320, 320);
        double deltaPitch = MinMax.clip(ballCenter.y - cameraCenter.y, -320, 320);

        yawAdjustment = MinMax.clip(yawController.calculate(deltaYaw, 0.0), -0.05, 0.05);
        pitchAdjustment = MinMax.clip(pitchController.calculate(deltaPitch, 0.0), -0.0025, 0.0025);

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
        DcMotor rightMotorDc1 = hardwareMap.get(DcMotor.class, "rightDrive1");
        DcMotor rightMotorDc2 = hardwareMap.get(DcMotor.class, "rightDrive2");
        DcMotor leftMotorDc1 = hardwareMap.get(DcMotor.class, "leftDrive1");
        DcMotor leftMotorDc2 = hardwareMap.get(DcMotor.class, "leftDrive2");
        DcMotor shooterMotor = hardwareMap.get(DcMotor.class,"shooter");
        Servo pusher = hardwareMap.get(Servo.class, "pusher");
        pusher.setPosition(1.0);

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

            if (gamepad1.b) {
                double right = -0.25;
                double left = 0.25;

                right += yawAdjustment * 200;
                left += yawAdjustment * 200;

                rightMotorDc1.setPower(right);
                rightMotorDc2.setPower(right);
                leftMotorDc1.setPower(left);
                leftMotorDc2.setPower(left);

                continue;
            }

            if (gamepad1.a) {
                if (!isShooterActive) {
                    isShooterActive = true;
                    new Thread(() -> {
                        shooterMotor.setPower(-1);
                        sleep(3_000);
                        for (int i = 0; i < 3; i++) {
                            pusher.setPosition(0.0);
                            sleep(500);

                            pusher.setPosition(1.0);
                            if (i != 2) {
                                sleep(500);
                            }
                        }

                        shooterMotor.setPower(0);
                        isShooterActive = false;
                    }).start();
                }
            }
            double multiplier = 0.5;
            if (gamepad1.right_trigger > 0) multiplier = 1.0;
            else if (gamepad1.left_trigger > 0) multiplier = 0.25;

            double right = gamepad1.right_stick_y * multiplier;
            double left = -gamepad1.left_stick_y * multiplier;

            rightMotorDc1.setPower(right);
            rightMotorDc2.setPower(right);
            leftMotorDc1.setPower(left);
            leftMotorDc2.setPower(left);

            telemetry.addData("cameraCenter", cameraCenter);
            telemetry.addData("ballCenter", ballCenter);
            telemetry.addData("avgBrightness", pipeline.getAvgBrightness());
            telemetry.update();

//            sleep(1_000);
        }
    }
}
