package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class GenericVision extends LinearOpMode {

    @Override
    public abstract void runOpMode();

    WebcamInitialize webcam = new WebcamInitialize();

    public GimbalOpenCv.MarkerPosition getMarkerPosition() {
        return webcam.getCurrentPosition();
    }


}
