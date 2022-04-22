package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class GenericVision extends LinearOpMode {

    @Override
    public abstract void runOpMode();

    public WebcamInitialize webcam = new WebcamInitialize();

    public MarkerDetectorPipeline.MarkerPosition getCurrentPosition() {
        return webcam.getCurrentPosition();
    }

    public void initialize() {
        while (!isStarted() && !isStopRequested()) {
            webcam.initialize(hardwareMap);
            telemetry.addData("position", getCurrentPosition());
            telemetry.update();
            sleep(100);
        }
    }

}
