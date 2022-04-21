package org.firstinspires.ftc.teamcode.Vision;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Vision Runner", group = "Test")
public class VisionRunner extends GenericVision{
    @Override
    public void runOpMode() {
        telemetry.addData("Pos", super.getMarkerPosition());
        telemetry.update();
    }
}
