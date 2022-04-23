package org.firstinspires.ftc.teamcode.WestCoastDrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Vision.GimbalMotion;

import me.wobblyyyy.pathfinder2.Pathfinder;
import me.wobblyyyy.pathfinder2.utils.Toggle;

@TeleOp (name = "GimbalTest", group = "Test")
public class GimbalTest extends LinearOpMode {
    private static GimbalMotion gimbal;
    private Toggle stickControl;
    private double targetYaw;
    private double targetPitch;
    private final Pathfinder pathfinder = Pathfinder.newSimulatedPathfinder(-0.05);

    @Override
    public void runOpMode() {
        gimbal = new GimbalMotion(hardwareMap.servo.get("webcamYaw"), hardwareMap.servo.get("webcamPitch"));
        gimbal.resetPosition(); // sets pitch and yaw to 0.5 (forward facing)
        stickControl = new Toggle(true);
        targetYaw = gimbal.getYawZero();
        targetPitch = gimbal.getPitchZero();

        waitForStart();

        if (isStopRequested()) return;

        pathfinder
                .getListenerManager()
                .bindButtonPress(() -> gamepad1.a, stickControl::toggle);

        while(opModeIsActive()) {
            pathfinder.tick();

            if (stickControl.getState()) {
                double left = (gamepad1.left_stick_y + 1) * 0.5; // yaw - left joystick
                double right = (gamepad1.right_stick_y + 1) * 0.5; // pitch - right joystick
                gimbal.setPosition(left, right);

            } else {
                gimbal.setPosition(targetYaw, targetPitch);

            }
        }
    }
}
