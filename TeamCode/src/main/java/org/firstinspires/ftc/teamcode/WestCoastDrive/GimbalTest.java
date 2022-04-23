package org.firstinspires.ftc.teamcode.WestCoastDrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "GimbalTest", group = "Test")
public class GimbalTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo yawServo = hardwareMap.servo.get("webcamYaw");
        Servo pitchServo = hardwareMap.servo.get("webcamPitch");

        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()) {
            double left = (gamepad1.left_stick_x + 1) * 0.5;
            double right = (gamepad1.right_stick_y + 1) * 0.5;

            telemetry.addData("Yaw Pos", left);
            telemetry.addData("Pitch Pos", right);
            telemetry.update();

            yawServo.setPosition(left);
            pitchServo.setPosition(right);
        }
    }
}
