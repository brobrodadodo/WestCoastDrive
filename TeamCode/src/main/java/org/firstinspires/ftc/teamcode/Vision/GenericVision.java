package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class GenericVision extends LinearOpMode {

    @Override
    public abstract void runOpMode();

    public WebcamInitialize webcam = new WebcamInitialize();

    public MarkerDetectorPipeline.MarkerPosition getCurrentPosition() {
        return webcam.getCurrentPosition();
    }

    public void initialize() {
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("position", getCurrentPosition());
            telemetry.update();
            sleep(100);
        }
    }

}
