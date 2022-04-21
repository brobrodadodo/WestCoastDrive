package org.firstinspires.ftc.teamcode.WestCoastDrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Util.GimbalOpenCv;
import org.firstinspires.ftc.teamcode.Util.WebcamInitialize;

public abstract class GenericVision extends LinearOpMode {

    @Override
    public abstract void runOpMode();

    public GimbalOpenCv.MarkerPosition getMarkerPosition() {
        return WebcamInitialize.getCurrentPosition();
    }
}
